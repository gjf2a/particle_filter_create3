use crossbeam_utils::atomic::AtomicCell;
use eframe::egui::{self, Context, Pos2, Ui, Vec2, Visuals};
use particle_filter::{Degrees, Noise, consistent::ConsistentParticleFilter};
use particle_filter_create3::{
    Noises,
    drivers::{ConsistentData, Estimate},
    grid_obstacles::GridObstacles,
    odometry_transcripts::Transcript,
};
use std::{env, sync::Arc, time::Instant};

pub fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 2 {
        println!("Usage: particle_filter_gui fileneme");
        return;
    }
    let transcript = Transcript::from_transcript(args[1].as_str()).unwrap();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(Vec2 { x: 800.0, y: 600.0 })
            .with_position(Pos2 { x: 50.0, y: 25.0 })
            .with_drag_and_drop(true),
        ..Default::default()
    };
    eframe::run_native(
        "Particle Filters",
        native_options,
        Box::new(|cc| {
            let mut app = MainApp::new(transcript);
            app.setup(&cc.egui_ctx);
            Ok(Box::new(app))
        }),
    )
    .unwrap();
}

#[derive(Clone)]
struct MainApp {
    transcript: Transcript,
    map_choice: MapChoice,
    num_particles: String,
    m_per_square: String,
    obst_noise_xy: String,
    obst_noise_theta: String,
    clear_noise_xy: String,
    clear_noise_theta: String,
    last_progress: Option<String>,
    progress: Arc<AtomicCell<Option<String>>>,
    results: Arc<AtomicCell<Option<ConsistentData>>>,
}

const FPS: f32 = 20.0;
const FRAME_INTERVAL: f32 = 1.0 / FPS;

impl eframe::App for MainApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Particle Filters");
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

#[derive(Copy, Clone, PartialEq, Eq, Default)]
enum MapChoice {
    #[default]
    FixedGrid,
    GrowingGrid,
}

impl MainApp {
    fn new(transcript: Transcript) -> Self {
        Self {
            transcript,
            m_per_square: "0.1".to_string(),
            map_choice: MapChoice::GrowingGrid,
            num_particles: "100".to_string(),
            clear_noise_xy: "7e-4".to_string(),
            clear_noise_theta: "2e-4".to_string(),
            obst_noise_xy: "0.016".to_string(),
            obst_noise_theta: "0.31".to_string(),
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

            ui.vertical(|ui| {
                ui.radio_value(
                    &mut self.map_choice,
                    MapChoice::FixedGrid,
                    "Fixed-Size Grid",
                );
                ui.radio_value(&mut self.map_choice, MapChoice::GrowingGrid, "Growing Grid");
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

            if let Some(progress) = &self.last_progress {
                ui.label(progress);
            }
        });
    }

    fn start(&self) -> anyhow::Result<()> {
        let num_particles = self.num_particles.parse::<usize>()?;
        let square_size_m = self.m_per_square.parse::<f64>()?;
        let noises = self.noises_from_ui()?;
        let transcript = self.transcript.clone();
        let which_one = self.map_choice;
        let progress = self.progress.clone();
        let results = self.results.clone();
        std::thread::spawn(move || match which_one {
            MapChoice::FixedGrid => todo!(),
            MapChoice::GrowingGrid => Self::growing_grid_loop(
                transcript,
                square_size_m,
                noises,
                num_particles,
                progress,
                results,
            ),
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
        square_size_m: f64,
        noises: Noises,
        num_particles: usize,
        progress: Arc<AtomicCell<Option<String>>>,
        results: Arc<AtomicCell<Option<ConsistentData>>>,
    ) {
        let start = Instant::now();
        results.store(None);
        let starting_map = GridObstacles::new(square_size_m, noises);
        let mut particle_filter = ConsistentParticleFilter::new(num_particles, &starting_map);
        for (i, sensor_info) in transcript.iter().enumerate() {
            let elapsed = Instant::now().duration_since(start);
            Self::send_progress(progress.clone(), i, transcript.len(), elapsed.as_secs_f64());
            particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
            if particle_filter.failed() {
                progress.store(Some(format!("Failed at iteration {i}")));
                break;
            }
        }
        results.store(Some(Self::pack_results(transcript, particle_filter)));
    }

    fn send_progress(
        progress: Arc<AtomicCell<Option<String>>>,
        i: usize,
        len: usize,
        elapsed: f64,
    ) {
        progress.store(Some(format!("{i}/{len} ({elapsed:.2}s)")));
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
            match results.outcome {
                Estimate::Failure(failure_iteration) => {
                    ui.label(format!("Failure Iteration: {failure_iteration}"));
                }
                Estimate::Success(closest_estimate, estimate_error, dist_to_actual, farthest_estimate) => {
                    ui.label(format!("Actual position: {}", results.actual));
                    ui.label(format!("Odometry pose: {}", results.odometry_pose));
                    ui.label(format!("Odometry error: {}", results.odometry_error));
                    ui.label(format!("Particle pose: {closest_estimate}"));
                    ui.label(format!("Particle error: {estimate_error}"));
                    ui.label(format!("Distance to actual: {dist_to_actual}"));
                    ui.label(format!("Farthest particle distance: {farthest_estimate}"));
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
}
