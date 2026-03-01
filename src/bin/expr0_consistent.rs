use particle_filter::{Degrees, Noise};
use particle_filter_create3::{
    Noises, drivers::consistent_expr, odometry_transcripts::Transcript,
};

fn main() -> anyhow::Result<()> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() == 4 {
        let transcript = Transcript::from_transcript(args[1].as_str())?;
        let num_particles = args[2].parse::<usize>()?;
        let square_size_m = args[3].parse::<f64>()?;
        let noises = Noises {
            odom: Noise {
                stdev_x_y: 0.0,
                stdev_angle: Degrees::new(0.0),
            },
            obst: Noise {
                stdev_x_y: 0.0,
                stdev_angle: Degrees::new(0.0),
            },
        };
        consistent_expr(square_size_m, noises, num_particles, &transcript);
    } else {
        println!("Usage: expr0_coherent transcript_filename num_particles square_size_m");
    }
    Ok(())
}
