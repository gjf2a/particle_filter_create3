use eframe::egui::{self, Context, Pos2, Vec2, Visuals};

pub fn main() {
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
}

impl MainApp {
    fn new() -> Self {
        Self {
            m_per_square: "0.1".to_string(),
            map_choice: MapChoice::Grid,
            num_particles: "100".to_string(),
            obstacles_only: false,
        }
    }

    fn setup(&mut self, ctx: &Context) {
        ctx.set_pixels_per_point(2.0);
        ctx.set_visuals(Visuals::light());
    }
}

impl eframe::App for MainApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Particle Filters");
            ui.vertical(|ui| {
                ui.checkbox(&mut self.obstacles_only, "Obstacles Only");
                ui.horizontal(|ui| {
                    ui.label("Particles");
                    ui.text_edit_singleline(&mut self.num_particles);
                });

                ui.horizontal(|ui| {
                    ui.label("Meters per square");
                    ui.text_edit_singleline(&mut self.m_per_square);
                });

                ui.vertical(|ui| {
                    ui.radio_value(&mut self.map_choice, MapChoice::Grid, "Grid");
                    ui.radio_value(&mut self.map_choice, MapChoice::Circle, "Circle");
                });
                if ui.button("Start").clicked() {}
            });
        });
    }
}
