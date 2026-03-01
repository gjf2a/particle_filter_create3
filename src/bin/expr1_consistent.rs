use particle_filter::{Degrees, Noise};
use particle_filter_create3::{
    Noises, drivers::consistent_driver, odometry_transcripts::Transcript,
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
                stdev_x_y: 0.16,
                stdev_angle: Degrees::new(3.1),
            },
        };
        consistent_driver(square_size_m, noises, num_particles, &transcript);
    } else {
        println!("Usage: expr1_coherent transcript_filename num_particles square_size_m");
    }
    Ok(())
}
