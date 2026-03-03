use bit_grid::BitGrid;
use crossbeam_utils::atomic::AtomicCell;
use eframe::egui::{self, Color32, Context, CornerRadius, Pos2, Rect, Ui, Vec2, Visuals};
use particle_filter::{
    Degrees, Noise, Radians, RobotPose,
    consistent::{ConsistentParticleFilter, SelectionStrategy},
};
use particle_filter_create3::{
    Noises,
    drivers::{ConsistentData, Estimate},
    grid_obstacles::{Cell, GridObstacles},
    odometry_transcripts::Transcript,
};
use std::{collections::HashSet, env, sync::Arc, time::Instant};

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
    last_progress: Option<(String, GridObstacles, RobotPose<Radians>)>,
    progress: Arc<AtomicCell<Option<(String, GridObstacles, RobotPose<Radians>)>>>,
    results: Arc<AtomicCell<Option<ConsistentData>>>,
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
                if let Some(results) = self.results.take() {
                    self.results.store(Some(results.clone()));
                    Self::render_results(ui, &results);
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
            last_progress: None,
            progress: Arc::new(AtomicCell::new(None)),
            results: Arc::new(AtomicCell::new(None)),
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

            if let Some(progress) = self.progress.take() {
                self.last_progress = Some(progress);
            }

            if let Some((progress, map, pose)) = &self.last_progress {
                ui.label(progress);
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
                ui.label(format!("open/all frontier/all space: {}/{}/{}", frontier.len(), all_frontier.len(), map.num_spaces()));
                Self::render_map(ui, map, *pose, &frontier);
            }
        });
    }

    fn start(&self) -> anyhow::Result<()> {
        let selection_strategy = self.selection_strategy;
        let num_particles = self.num_particles.parse::<usize>()?;
        let square_size_m = self.m_per_square.parse::<f64>()?;
        let noises = self.noises_from_ui()?;
        let transcript = self.transcript.clone();
        let progress = self.progress.clone();
        let results = self.results.clone();
        std::thread::spawn(move || {
            Self::growing_grid_loop(
                transcript,
                selection_strategy,
                square_size_m,
                noises,
                num_particles,
                progress,
                results,
            )
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

    fn growing_grid_loop(
        transcript: Transcript,
        selection_strategy: SelectionStrategy,
        square_size_m: f64,
        noises: Noises,
        num_particles: usize,
        progress: Arc<AtomicCell<Option<(String, GridObstacles, RobotPose<Radians>)>>>,
        results: Arc<AtomicCell<Option<ConsistentData>>>,
    ) {
        let start = Instant::now();
        results.store(None);
        let starting_map = GridObstacles::new(square_size_m, noises);
        let mut particle_filter =
            ConsistentParticleFilter::new(num_particles, &starting_map, selection_strategy);
        for (i, sensor_info) in transcript.iter().enumerate() {
            let particle = particle_filter.particles().next().unwrap();
            let map = particle.map().clone();
            let pose = particle.estimated_pose();
            let elapsed = Instant::now().duration_since(start);
            Self::send_progress(
                progress.clone(),
                i,
                transcript.len(),
                elapsed.as_secs_f64(),
                &map,
                pose,
            );
            particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
            if let Some(failure) = particle_filter.example_failure() {
                Self::send_progress(
                    progress,
                    i,
                    transcript.len(),
                    elapsed.as_secs_f64(),
                    &failure.map(),
                    failure.estimated_pose(),
                );
                break;
            }
        }
        results.store(Some(Self::pack_results(transcript, particle_filter)));
    }

    fn send_progress(
        progress: Arc<AtomicCell<Option<(String, GridObstacles, RobotPose<Radians>)>>>,
        i: usize,
        len: usize,
        elapsed: f64,
        map: &GridObstacles,
        pose: RobotPose<Radians>,
    ) {
        progress.store(Some((
            format!(
                "{i}/{len} ({elapsed:.2}s; {:.1}ms/iteration)",
                1000.0 * elapsed / i as f64
            ),
            map.clone(),
            pose,
        )));
    }

    fn pack_results(
        transcript: Transcript,
        particle_filter: ConsistentParticleFilter<GridObstacles>,
    ) -> ConsistentData {
        let stats = particle_filter.stats();
        ConsistentData::new(&transcript, &particle_filter, &stats)
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
            ui.label(format!("Actual position: {}", results.actual));
            ui.label(format!("Odometry pose: {}", results.odometry_pose));
            ui.label(format!("Odometry error: {}", results.odometry_error));
            ui.label(format!(
                "Odometry to actual: {:.2}m",
                results.odometry_distance
            ));
            match &results.outcome {
                Estimate::Failure(failure_iteration) => {
                    ui.label(format!("Failure Iteration: {failure_iteration}"));
                }
                Estimate::Success(data) => {
                    ui.label(format!("Particle pose: {}", data.closest_estimate));
                    ui.label(format!("Particle error: {}", data.closest_error));
                    ui.label(format!("Particle to actual: {:.2}m", data.estimate_to_actual));
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
        });
    }

    fn render_map(ui: &mut Ui, map: &GridObstacles, pose: RobotPose<Radians>, frontier: &HashSet<(i64,i64)>) {
        let (response, painter) = ui.allocate_painter(
            Vec2::new(
                map.width() as f32 * MAP_CELL_SIZE,
                map.height() as f32 * MAP_CELL_SIZE,
            ),
            egui::Sense::hover(),
        );
        let shadow = map.robot_shadow(pose);
        let response_rect = response.rect;
        let (min_x, min_y) = map.upper_left_x_y();
        for (x, y, cell) in map.points() {
            let x_rect = ((x - min_x) as f32) * MAP_CELL_SIZE + response_rect.left();
            let y_rect = ((y - min_y) as f32) * MAP_CELL_SIZE + response_rect.top();
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
            let fill_color = if cell == Cell::Space {
                if frontier.contains(&(x, y)) {
                    Color32::CYAN
                } else if shadow.is_set(x, y) {
                    Color32::GRAY
                } else {
                    cell.color()
                }
            } else {
                cell.color()
            };
            painter.rect_filled(rect, CornerRadius::ZERO, fill_color);
        }
    }
}
