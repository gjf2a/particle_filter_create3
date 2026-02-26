use eframe::egui::{self, Context, Pos2, Ui, Vec2, Visuals};
use particle_filter_create3::odometry_transcripts::Transcript;
use std::env;

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
            let mut app = MainApp::new();
            app.setup(&cc.egui_ctx);
            Ok(Box::new(app))
        }),
    )
    .unwrap();
}

#[derive(Copy, Clone, PartialEq, Eq, Default)]
enum MapChoice {
    #[default]
    Grid,
    Circle,
}

#[derive(Default, Clone)]
struct MainApp {
    map_choice: MapChoice,
    num_particles: String,
    m_per_square: String,
    obstacles_only: bool,
    obst_noise_xy: String,
    obst_noise_theta: String,
    clear_noise_xy: String,
    clear_noise_theta: String,
}

impl MainApp {
    fn new() -> Self {
        Self {
            m_per_square: "0.1".to_string(),
            map_choice: MapChoice::Grid,
            num_particles: "100".to_string(),
            obstacles_only: false,
            clear_noise_xy: "7e-4".to_string(),
            clear_noise_theta: "2e-4".to_string(),
            obst_noise_xy: "0.16".to_string(),
            obst_noise_theta: "3.1".to_string(),
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
            ui.checkbox(&mut self.obstacles_only, "Obstacles Only");
            if !self.obstacles_only {
                Self::noise_entry(
                    ui,
                    "Clear Noise",
                    &mut self.clear_noise_xy,
                    &mut self.clear_noise_theta,
                );
            }
            ui.horizontal(|ui| {
                ui.label("Particles");
                ui.text_edit_singleline(&mut self.num_particles);
            });

            ui.vertical(|ui| {
                ui.radio_value(&mut self.map_choice, MapChoice::Grid, "Grid");
                ui.radio_value(&mut self.map_choice, MapChoice::Circle, "Circle");
            });

            if let MapChoice::Grid = self.map_choice {
                ui.horizontal(|ui| {
                    ui.label("Meters per square");
                    ui.text_edit_singleline(&mut self.m_per_square);
                });
            }

            if ui.button("Start").clicked() {}
        });
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
}

impl eframe::App for MainApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Particle Filters");
            ui.horizontal(|ui| {
                self.render_settings(ui);
                ui.vertical(|ui| {});
            });
        });
    }
}
