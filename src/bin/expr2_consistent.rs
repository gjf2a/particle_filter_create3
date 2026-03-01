use particle_filter::{Degrees, Noise};
use particle_filter_create3::{
    Noises, drivers::{consistent_expr, consistent_report, inconsistent_report}, odometry_transcripts::Transcript,
};

fn main() -> anyhow::Result<()> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() == 4 {
        let transcript = Transcript::from_transcript(args[1].as_str())?;
        let num_particles = args[2].parse::<usize>()?;
        let square_size_m = args[3].parse::<f64>()?;
        let noises = Noises {
            odom: Noise {
                stdev_x_y: 7e-4,
                stdev_angle: Degrees::new(2e-4),
            },
            obst: Noise {
                stdev_x_y: 0.016,
                stdev_angle: Degrees::new(0.31),
            },
        };
        let particle_filter = consistent_expr(square_size_m, noises, num_particles, &transcript);
        consistent_report(&transcript, &particle_filter);
        inconsistent_report(&particle_filter.stats(), particle_filter.failed());
    } else {
        println!("Usage: expr2_consistent transcript_filename num_particles square_size_m");
    }
    Ok(())
}
