use bit_grid::{
    BitGrid,
    angle::{Degrees, Radians},
    point::GridPoint,
    pose::RobotPose,
};
use chrono::Local;
use crossbeam_utils::atomic::AtomicCell;
use eframe::egui::{self, Color32, Context, CornerRadius, Painter, Pos2, Rect, Ui, Vec2, Visuals};
use enum_iterator::{Sequence, all};
use particle_filter::{
    BitGridMap, Cell, Noise, Noises, Particle, ParticleFilter, ParticleFilterSettings,
    SelectionStrategy, WeightStrategy,
    path_plan::paths_from,
};
use particle_filter_create3::{
    CREATE3_RADIUS, cell2color,
    drivers::{MultiRunData, OneRunData, SuccessData},
    odometry_transcripts::Transcript,
};
use std::{
    env,
    fmt::Debug,
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
    weight_strategy: WeightStrategy,
    num_particles: String,
    m_per_square: String,
    obst_noise_xy: String,
    obst_noise_theta: String,
    clear_noise_xy: String,
    clear_noise_theta: String,
    num_exprs: String,
    thread_running: Arc<AtomicCell<bool>>,
    status: Arc<Mutex<Option<CurrentData>>>,
    expr_data: Arc<Mutex<Option<MultiRunData>>>,
    current_expr: Arc<AtomicCell<Option<usize>>>,
    show_waypoints: PathsOut,
}

#[derive(Clone)]
struct CurrentData {
    message: String,
    map: BitGridMap,
    results: Option<OneRunData>,
    current_particle: usize,
    pose: RobotPose<Radians>,
}

#[derive(PartialEq, Eq, Copy, Clone, Sequence, Debug)]
enum PathsOut {
    None,
    ShortestPath,
    AllPaths,
}

impl PathsOut {
    fn path_out_grid(&self, map: &BitGridMap, start: RobotPose<Radians>) -> BitGrid {
        match self {
            Self::None => BitGrid::default(),
            Self::ShortestPath => {
                let paths_back = paths_from(map, start);
                match paths_back.shortest_path() {
                    None => BitGrid::default(),
                    Some(shortest) => shortest.iter().collect(),
                }
            }
            Self::AllPaths => {
                let paths_back = paths_from(map, start);
                let mut grid = BitGrid::default();
                for leaf in paths_back.leaves().ones() {
                    for square in paths_back.path_to_start(leaf) {
                        grid.set(square, true);
                    }
                }
                grid
            }
        }
    }
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
                self.render_choices(ui);
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
            selection_strategy: SelectionStrategy::RankProportion,
            weight_strategy: WeightStrategy::MinPose,
            m_per_square: "0.1".to_string(),
            num_particles: "1000".to_string(),
            clear_noise_xy: "7e-4".to_string(),
            clear_noise_theta: "2e-4".to_string(),
            obst_noise_xy: "0.032".to_string(),
            obst_noise_theta: "0.62".to_string(),
            num_exprs: "100".to_string(),
            status: Arc::new(Mutex::new(None)),
            expr_data: Arc::new(Mutex::new(None)),
            thread_running: Arc::new(AtomicCell::new(false)),
            current_expr: Arc::new(AtomicCell::new(None)),
            show_waypoints: PathsOut::None,
        }
    }

    fn thread_running(&self) -> bool {
        self.thread_running.load()
    }

    fn settings(&self) -> anyhow::Result<ParticleFilterSettings> {
        Ok(ParticleFilterSettings {
            noises: self.noises_from_ui()?,
            num_particles: self.num_particles.parse::<usize>()?,
            square_size_m: self.m_per_square.parse::<f64>()?,
            robot_radius_m: CREATE3_RADIUS,
            selection_strategy: self.selection_strategy,
            weight_strategy: self.weight_strategy,
        })
    }

    fn setup(&mut self, ctx: &Context) {
        ctx.set_pixels_per_point(2.0);
        ctx.set_visuals(Visuals::light());
    }

    fn limited_text_edit(ui: &mut Ui, target: &mut String) {
        ui.add_sized(egui::vec2(100.0, 20.0), egui::TextEdit::singleline(target));
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
                Self::limited_text_edit(ui, &mut self.num_particles);
            });

            ui.horizontal(|ui| {
                ui.label("Meters per square");
                Self::limited_text_edit(ui, &mut self.m_per_square);
            });

            if !self.thread_running() {
                ui.horizontal(|ui| {
                    if ui.button("Start").clicked() {
                        if let Err(e) = self.start() {
                            ui.label(format!("{e}"));
                        }
                    }
                });
            }

            self.assess_progress(ui);
        });
    }

    fn render_choices(&mut self, ui: &mut Ui) {
        ui.vertical(|ui| {
            ui.heading("View Paths Out");
            Self::render_radios(ui, &mut self.show_waypoints, all::<PathsOut>());
            ui.heading("Selection Strategy");
            Self::render_radios(ui, &mut self.selection_strategy, all::<SelectionStrategy>());
            ui.heading("Weight Calculation");
            Self::render_radios(ui, &mut self.weight_strategy, all::<WeightStrategy>());
            if self.thread_running() {
                self.current_expr.load().map(|current_expr| {
                    ui.label(format!("{current_expr}/{}", self.num_exprs));
                });
            } else {
                ui.horizontal(|ui| {
                    ui.label("Number of Runs");
                    Self::limited_text_edit(ui, &mut self.num_exprs);
                });
                if ui.button("Experiments").clicked() {
                    if let Err(e) = self.run_experiments() {
                        ui.label(format!("Experiments error: {e}"));
                    }
                }
            }
            self.expr_success_failure(ui);
        });
    }

    fn expr_success_failure(&self, ui: &mut Ui) {
        let expr_data = self.expr_data.lock().unwrap();
        if let Some(expr_data) = expr_data.as_ref() {
            ui.label(format!(
                "{} successes, {} failures",
                expr_data.num_successes(),
                expr_data.num_failures()
            ));
        }
    }

    fn run_experiments(&self) -> anyhow::Result<()> {
        let num_exprs = self.num_exprs.parse::<usize>()?;
        let settings = self.settings()?;
        let runner = self.make_runner()?;
        let status = self.status.clone();
        let expr_data = self.expr_data.clone();
        let thread_running = self.thread_running.clone();
        let current_expr = self.current_expr.clone();
        let filename = self.filename.clone();
        std::thread::spawn(move || {
            {
                let mut expr_data = expr_data.lock().unwrap();
                *expr_data = None;
            }
            thread_running.store(true);
            for current in 0..num_exprs {
                current_expr.store(Some(current + 1));
                let mut runner = runner.clone();
                runner.run();
                let status = status.lock().unwrap();
                if let Some(status) = status.as_ref() {
                    if let Some(data_from_run) = &status.results {
                        let mut expr_data = expr_data.lock().unwrap();
                        match expr_data.as_mut() {
                            Some(expr_data) => {
                                expr_data.data.push(data_from_run.clone());
                            }
                            None => {
                                *expr_data = Some(MultiRunData {
                                    data: vec![data_from_run.clone()],
                                    settings,
                                });
                            }
                        }
                    }
                }
            }
            current_expr.store(None);
            thread_running.store(false);
            let now = Local::now();
            let csv_filename = format!("{filename}_{}.csv", now.format("%Y_%m_%d_%H_%M_%S"));
            let csv = {
                let expr_data = expr_data.lock().unwrap();
                expr_data
                    .as_ref()
                    .map_or(String::new(), |data| data.to_csv())
            };
            if let Err(e) = std::fs::write(csv_filename.as_str(), csv) {
                println!("File I/O problem: {e}");
            }
        });
        Ok(())
    }

    fn render_radios<S: Iterator<Item = T>, T: Eq + Copy + Debug>(
        ui: &mut Ui,
        state: &mut T,
        items: S,
    ) {
        ui.vertical(|ui| {
            for item in items {
                ui.radio_value(state, item, format!("{item:?}"));
            }
        });
    }

    fn assess_progress(&self, ui: &mut Ui) {
        let mut status = self.status.lock().unwrap();
        if let Some(status) = &mut *status {
            if let Some(data) = status.results.clone() {
                self.render_completed(ui, &data.clone(), status);
            } else {
                self.render_progress(ui, &status.message, &status.map);
                Self::render_map(ui, &status.map, status.pose, self.show_waypoints);
            }
        }
    }

    fn render_completed(&self, ui: &mut Ui, data: &OneRunData, status: &mut CurrentData) {
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
        );
        let particle = &data.particle_filter[status.current_particle];
        Self::render_map(
            ui,
            particle.map(),
            particle.estimated_pose(),
            self.show_waypoints,
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

    fn render_progress(&self, ui: &mut Ui, msg: &str, map: &BitGridMap) {
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
    }

    fn make_runner(&self) -> anyhow::Result<ParticleFilterRunner> {
        Ok(ParticleFilterRunner {
            transcript: self.transcript.clone(),
            settings: self.settings()?,
            status: self.status.clone(),
            duration: 0.0,
            mean_iteration_time: 0.0,
            max_iteration_time: 0.0,
        })
    }

    fn start(&self) -> anyhow::Result<()> {
        let mut runner = self.make_runner()?;
        let thread_running = self.thread_running.clone();
        std::thread::spawn(move || {
            thread_running.store(true);
            runner.run();
            thread_running.store(false);
        });
        Ok(())
    }

    fn noises_from_ui(&self) -> anyhow::Result<Noises> {
        Ok(Noises {
            clear: Noise {
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
                Self::limited_text_edit(ui, entry_x_y);
            });
            ui.horizontal(|ui| {
                ui.label("theta");
                Self::limited_text_edit(ui, entry_theta);
            });
        });
    }

    fn render_results(ui: &mut Ui, results: &OneRunData) {
        ui.vertical(|ui| {
            render_outcome(ui, &results.outcome);

            ui.label(format!("Actual position: {}", results.actual));
            for line in results.odometry_report.report("Odometry") {
                ui.label(line);
            }

            render_inconsistencies(ui, results);
        });
    }

    fn render_map(
        ui: &mut Ui,
        map: &BitGridMap,
        pose: RobotPose<Radians>,
        show_paths_out: PathsOut,
    ) {
        let bounds = map.bordered_bounding_box();
        let (response, painter) = ui.allocate_painter(
            Vec2::new(
                bounds.height() as f32 * MAP_CELL_SIZE,
                bounds.width() as f32 * MAP_CELL_SIZE,
            ),
            egui::Sense::hover(),
        );
        let shadow = map.robot_shadow(pose);
        let response_rect = response.rect;
        let frontier = map.open_frontier_spaces();
        let paths_out = show_paths_out.path_out_grid(map, pose);
        for p in bounds.coord_iter() {
            let cell = map.cell_for(&p);
            let x_rect = ((p[1] - bounds.min()[1]) as f32) * MAP_CELL_SIZE + response_rect.left();
            let y_rect = ((p[0] - bounds.min()[0]) as f32) * MAP_CELL_SIZE + response_rect.top();
            let color = cell_color(&paths_out, &frontier, &shadow, cell, p);
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

fn cell_color(
    waypoints: &BitGrid,
    frontier: &BitGrid,
    shadow: &BitGrid,
    cell: Cell,
    p: GridPoint,
) -> Color32 {
    if waypoints.get(&p) && (cell == Cell::Space || cell == Cell::Unvisited) {
        Color32::ORANGE
    } else if cell == Cell::Space {
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

fn render_inconsistencies(ui: &mut Ui, results: &OneRunData) {
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

#[derive(Clone)]
pub struct ParticleFilterRunner {
    transcript: Transcript,
    settings: ParticleFilterSettings,
    status: Arc<Mutex<Option<CurrentData>>>,
    duration: f64,
    mean_iteration_time: f64,
    max_iteration_time: f64,
}

impl ParticleFilterRunner {
    pub fn run(&mut self) {
        let start = Instant::now();
        self.reset_status();
        let mut particle_filter = ParticleFilter::new(self.settings);
        let transcript = self.transcript.clone();
        let mut num_completed = 0;
        for (i, sensor_info) in transcript.iter().enumerate() {
            let particle = particle_filter.particles().next().unwrap();
            let elapsed = Instant::now().duration_since(start);
            self.send_progress(i, elapsed, particle);
            particle_filter.iterate(
                sensor_info.odometry(),
                sensor_info.obstacles().map(|bump| bump.obstacle_at()),
            );
            num_completed = i + 1;
            if let Some(failure) = particle_filter.example_failure() {
                self.send_progress(i, elapsed, &failure);
                break;
            }
        }
        self.completion_status(num_completed, &particle_filter);
    }

    fn reset_status(&self) {
        let mut status = self.status.lock().unwrap();
        if let Some(status) = &mut *status {
            status.results = None;
        }
    }

    fn completion_status(&self, final_iteration: usize, particle_filter: &ParticleFilter) {
        let stats = particle_filter.stats();
        let packed_results = OneRunData::new(
            &self.transcript,
            &particle_filter,
            &stats,
            self.duration,
            self.mean_iteration_time,
            self.max_iteration_time,
            final_iteration,
        );
        let mut status = self.status.lock().unwrap();
        if let Some(status) = &mut *status {
            status.results = Some(packed_results);
        }
    }

    pub fn send_progress(&mut self, i: usize, elapsed: Duration, particle: &Particle) {
        self.duration = elapsed.as_secs_f64();
        self.mean_iteration_time = 1000.0 * self.duration / i as f64;
        if i > 0 && self.mean_iteration_time > self.max_iteration_time {
            self.max_iteration_time = self.mean_iteration_time;
        }
        let message = format!(
            "{}/{} ({:.2}s; {:.1}ms/iteration; longest {:.2}ms)",
            i + 1,
            self.transcript.len(),
            self.duration,
            self.mean_iteration_time,
            self.max_iteration_time,
        );
        match self.status.lock() {
            Ok(mut status) => {
                let results = status.as_ref().and_then(|s| s.results.clone());
                *status = Some(CurrentData {
                    message,
                    map: particle.map().clone(),
                    pose: particle.estimated_pose(),
                    results,
                    current_particle: 0,
                });
            }
            Err(e) => println!("Thread error: {e}"),
        }
    }
}
