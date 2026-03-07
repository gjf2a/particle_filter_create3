use bit_grid::{
    BitGrid,
    angle::{Degrees, Radians},
    point::GridPoint,
    pose::RobotPose,
};
use eframe::egui::{self, Color32, Context, CornerRadius, Painter, Pos2, Rect, Ui, Vec2, Visuals};

use particle_filter::{
    ConsistentParticleFilter, Noise, SelectionStrategy,
    bit_grid_map::{BitGridMap, Cell},
};
use particle_filter_create3::{
    Create3Info, Noises, cell2color,
    drivers::{ConsistentData, SuccessData},
    odometry_transcripts::Transcript,
};
use std::{
    env,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

const MAP_CELL_SIZE: f32 = 3.0;

pub fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 2 {
        println!("Usage: particle_filter_gui fileneme");
        return;
    }
    let transcript_filename = args[1].as_str();
    let transcript = Transcript::from_transcript(transcript_filename).unwrap();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(Vec2 {
                x: 1200.0,
                y: 900.0,
            })
            .with_position(Pos2 { x: 50.0, y: 25.0 })
            .with_drag_and_drop(true),
        ..Default::default()
    };
    eframe::run_native(
        "Particle Filters",
        native_options,
        Box::new(|cc| {
            let mut app = MainApp::new(transcript_filename, transcript);
            app.setup(&cc.egui_ctx);
            Ok(Box::new(app))
        }),
    )
    .unwrap();
}

#[derive(Clone)]
struct MainApp {
    filename: String,
    transcript: Transcript,
    selection_strategy: SelectionStrategy,
    num_particles: String,
    m_per_square: String,
    obst_noise_xy: String,
    obst_noise_theta: String,
    clear_noise_xy: String,
    clear_noise_theta: String,
    status: Arc<Mutex<Option<CurrentData>>>,
}

#[derive(Clone)]
struct CurrentData {
    message: String,
    map: BitGridMap,
    pose: RobotPose<Radians>,
    results: Option<ConsistentData>,
    current_particle: usize,
}

const FPS: f32 = 20.0;
const FRAME_INTERVAL: f32 = 1.0 / FPS;

impl eframe::App for MainApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading(format!(
                "Particle Filter: {}; Duration {:.2}s",
                self.filename,
                self.transcript.total_time_seconds()
            ));
            ui.horizontal(|ui| {
                self.render_settings(ui);
                let status = self.status.lock().unwrap();
                if let Some(status) = &*status {
                    if let Some(results) = &status.results {
                        Self::render_results(ui, results);
                    }
                }
            });
            ctx.request_repaint_after_secs(FRAME_INTERVAL);
        });
    }
}

impl MainApp {
    fn new(filename: &str, transcript: Transcript) -> Self {
        Self {
            filename: filename.to_string(),
            transcript,
            selection_strategy: SelectionStrategy::DistanceWeight,
            m_per_square: "0.1".to_string(),
            num_particles: "1000".to_string(),
            clear_noise_xy: "7e-4".to_string(),
            clear_noise_theta: "2e-4".to_string(),
            obst_noise_xy: "0.032".to_string(),
            obst_noise_theta: "0.62".to_string(),
            status: Arc::new(Mutex::new(None)),
        }
    }

    fn setup(&mut self, ctx: &Context) {
        ctx.set_pixels_per_point(2.0);
        ctx.set_visuals(Visuals::light());
    }

    fn render_settings(&mut self, ui: &mut Ui) {
        ui.vertical(|ui| {
            Self::noise_entry(
                ui,
                "Collision Noise",
                &mut self.obst_noise_xy,
                &mut self.obst_noise_theta,
            );
            Self::noise_entry(
                ui,
                "Clear Noise",
                &mut self.clear_noise_xy,
                &mut self.clear_noise_theta,
            );
            ui.horizontal(|ui| {
                ui.label("Particles");
                ui.text_edit_singleline(&mut self.num_particles);
            });

            ui.horizontal(|ui| {
                ui.label("Meters per square");
                ui.text_edit_singleline(&mut self.m_per_square);
            });

            if ui.button("Start").clicked() {
                if let Err(e) = self.start() {
                    ui.label(format!("{e}"));
                }
            }

            self.assess_progress(ui);
        });
    }

    fn assess_progress(&self, ui: &mut Ui) {
        let mut status = self.status.lock().unwrap();
        if let Some(status) = &mut *status {
            if let Some(data) = &status.results {
                self.render_completed(ui, &data.clone(), status);
            } else {
                self.render_progress(ui, &status.message, &status.map, &status.pose);
            }
        }
    }

    fn render_completed(&self, ui: &mut Ui, data: &ConsistentData, status: &mut CurrentData) {
        let num_particles = data.particle_filter.len();
        let msg = match &data.outcome {
            None => format!("Failed"),
            Some(estimate) => {
                Self::render_map_selector(ui, status, num_particles, estimate.closest_rank);
                format!("Success")
            }
        };
        self.render_progress(
            ui,
            format!("{msg} {}", status.message).as_str(),
            data.particle_filter[status.current_particle].map(),
            &data.particle_filter[status.current_particle].estimated_pose(),
        );
    }

    fn render_map_selector(
        ui: &mut Ui,
        status: &mut CurrentData,
        num_particles: usize,
        closest_rank: usize,
    ) {
        ui.horizontal(|ui| {
            if ui.button("<").clicked() {
                if status.current_particle == 0 {
                    status.current_particle = num_particles - 1;
                } else {
                    status.current_particle -= 1;
                }
            }
            ui.label(format!("{}", status.current_particle + 1));
            if ui.button(">").clicked() {
                let right = status.current_particle + 1;
                status.current_particle = if right == num_particles { 0 } else { right };
            }
            if ui.button("Closest").clicked() {
                status.current_particle = closest_rank - 1;
            }
            if ui.button("Best").clicked() {
                status.current_particle = 0;
            }
        });
    }

    fn render_progress(&self, ui: &mut Ui, msg: &str, map: &BitGridMap, pose: &RobotPose<Radians>) {
        ui.label(msg);
        ui.label(format!(
            "{} x {} squares: {} words",
            map.width(),
            map.height(),
            map.map_words_used()
        ));
        let whm = map.width_height_meters();
        ui.label(format!("{:.1}m x {:.1}m", whm[0], whm[1]));
        let frontier = map.open_frontier_spaces();
        let all_frontier = map.all_frontier_spaces();
        ui.label(format!(
            "open/all frontier/all space: {}/{}/{}",
            frontier.count_ones(),
            all_frontier.count_ones(),
            map.num_spaces()
        ));
        Self::render_map(ui, map, *pose, &frontier);
    }

    fn start(&self) -> anyhow::Result<()> {
        let runner = ParticleFilterRunner {
            transcript: self.transcript.clone(),
            selection_strategy: self.selection_strategy,
            square_size_m: self.m_per_square.parse::<f64>()?,
            noises: self.noises_from_ui()?,
            num_particles: self.num_particles.parse::<usize>()?,
            status: self.status.clone(),
        };
        std::thread::spawn(move || {
            runner.run();
        });
        Ok(())
    }

    fn noises_from_ui(&self) -> anyhow::Result<Noises> {
        Ok(Noises {
            odom: Noise {
                stdev_x_y: self.clear_noise_xy.parse::<f64>()?,
                stdev_angle: Degrees::new(self.clear_noise_theta.parse::<f64>()?),
            },
            obst: Noise {
                stdev_x_y: self.obst_noise_xy.parse::<f64>()?,
                stdev_angle: Degrees::new(self.obst_noise_theta.parse::<f64>()?),
            },
        })
    }

    fn noise_entry(ui: &mut Ui, header: &str, entry_x_y: &mut String, entry_theta: &mut String) {
        ui.vertical(|ui| {
            ui.heading(header);
            ui.horizontal(|ui| {
                ui.label("x-y");
                ui.text_edit_singleline(entry_x_y);
            });
            ui.horizontal(|ui| {
                ui.label("theta");
                ui.text_edit_singleline(entry_theta);
            });
        });
    }

    fn render_results(ui: &mut Ui, results: &ConsistentData) {
        ui.vertical(|ui| {
            render_outcome(ui, &results.outcome);

            ui.label(format!("Actual position: {}", results.actual));
            for line in results.odometry_report.report("Odometry") {
                ui.label(line);
            }

            render_inconsistencies(ui, results);
        });
    }

    fn render_map(ui: &mut Ui, map: &BitGridMap, pose: RobotPose<Radians>, frontier: &BitGrid) {
        let (response, painter) = ui.allocate_painter(
            Vec2::new(
                map.height() as f32 * MAP_CELL_SIZE,
                map.width() as f32 * MAP_CELL_SIZE,
            ),
            egui::Sense::hover(),
        );
        let shadow = map.robot_shadow(pose);
        let response_rect = response.rect;
        let bb = map.bounding_box();
        for (p, cell) in map.points() {
            let x_rect = ((p[1] - bb.min()[1]) as f32) * MAP_CELL_SIZE + response_rect.left();
            let y_rect = ((p[0] - bb.min()[0]) as f32) * MAP_CELL_SIZE + response_rect.top();
            let color = cell_color(frontier, &shadow, cell, p);
            paint_cell(&painter, x_rect, y_rect, color);
        }
    }
}

fn paint_cell(painter: &Painter, x_rect: f32, y_rect: f32, color: Color32) {
    let rect = Rect::from_min_max(
        Pos2 {
            x: x_rect,
            y: y_rect,
        },
        Pos2 {
            x: x_rect + MAP_CELL_SIZE,
            y: y_rect + MAP_CELL_SIZE,
        },
    );
    painter.rect_filled(rect, CornerRadius::ZERO, color);
}

fn cell_color(frontier: &BitGrid, shadow: &BitGrid, cell: Cell, p: GridPoint) -> Color32 {
    if cell == Cell::Space {
        if frontier.get(&p) {
            Color32::CYAN
        } else if shadow.get(&p) {
            Color32::GRAY
        } else {
            cell2color(&cell)
        }
    } else {
        cell2color(&cell)
    }
}

fn render_outcome(ui: &mut Ui, outcome: &Option<SuccessData>) {
    match outcome {
        None => {
            ui.label(format!("Failure"));
        }
        Some(data) => {
            for line in data.best_particle_to_actual.report("Best-particle") {
                ui.label(line);
            }
            ui.label(format!("Closest-particle rank: {}", data.closest_rank));
            for line in data.closest_to_actual.report("Closest-particle") {
                ui.label(line);
            }
            ui.label(format!(
                "Farthest particle distance: {:.2}m",
                data.farthest_to_actual
            ));
            let (all_frontier, open_frontier, ratio) = data.all_and_open_frontier_counts();
            ui.label(format!(
                "Frontier counts: {open_frontier}/{all_frontier} ({:.2}%)",
                ratio * 100.0
            ));
        }
    }
}

fn render_inconsistencies(ui: &mut Ui, results: &ConsistentData) {
    ui.label(format!(
        "Iterations w/inconsistencies: {}",
        results.iterations_with_inconsistencies
    ));
    ui.label(format!(
        "Total inconsistencies: {}",
        results.total_inconsistencies
    ));
    ui.label(format!(
        "Total obstacle/space: {}",
        results.obstacle_space_issues
    ));
    ui.label(format!(
        "Total discontinuity: {}",
        results.discontinuity_issues
    ));
}

pub struct ParticleFilterRunner {
    transcript: Transcript,
    selection_strategy: SelectionStrategy,
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    status: Arc<Mutex<Option<CurrentData>>>,
}

impl ParticleFilterRunner {
    pub fn run(&self) {
        let start = Instant::now();
        self.reset_status();
        let mut particle_filter = ConsistentParticleFilter::new(
            self.num_particles,
            self.square_size_m,
            &Create3Info::new(self.noises),
            self.selection_strategy,
        );
        for (i, sensor_info) in self.transcript.iter().enumerate() {
            let particle = particle_filter.particles().next().unwrap();
            let map = particle.map().clone();
            let pose = particle.estimated_pose();
            let elapsed = Instant::now().duration_since(start);
            self.send_progress(i, elapsed, pose, &map);
            particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
            if let Some(failure) = particle_filter.example_failure() {
                self.send_progress(i, elapsed, failure.estimated_pose(), &failure.map());
                break;
            }
        }
        self.completion_status(&particle_filter);
    }

    fn reset_status(&self) {
        let mut status = self.status.lock().unwrap();
        if let Some(status) = &mut *status {
            status.results = None;
        }
    }

    fn completion_status(&self, particle_filter: &ConsistentParticleFilter<Create3Info>) {
        let stats = particle_filter.stats();
        let packed_results = ConsistentData::new(&self.transcript, &particle_filter, &stats);
        let mut status = self.status.lock().unwrap();
        if let Some(status) = &mut *status {
            status.results = Some(packed_results);
        }
    }

    pub fn send_progress(
        &self,
        i: usize,
        elapsed: Duration,
        pose: RobotPose<Radians>,
        map: &BitGridMap,
    ) {
        let elapsed = elapsed.as_secs_f64();
        let iteration = 1000.0 * elapsed / i as f64;
        let message = format!(
            "{}/{} ({elapsed:.2}s; {iteration:.1}ms/iteration)",
            i + 1,
            self.transcript.len()
        );
        let mut status = self.status.lock().unwrap();
        let results = status.as_ref().and_then(|s| s.results.clone());
        *status = Some(CurrentData {
            message,
            map: map.clone(),
            pose,
            results,
            current_particle: 0,
        });
    }
}
