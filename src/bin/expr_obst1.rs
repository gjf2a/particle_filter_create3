use particle_filter::{Degrees, Noise};
use particle_filter_create3::{Noises, drivers::obstacle_only_updates, from_transcript};

fn main() -> anyhow::Result<()> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() == 3 {
        let transcript = from_transcript(args[1].as_str())?;
        let num_particles = args[2].parse::<usize>()?;
        let noises = Noises {
            odom: Noise {
                stdev_x_y: 0.0,
                stdev_angle: Degrees::new(0.0),
            },
            obst: Noise {
                stdev_x_y: 0.016,
                stdev_angle: Degrees::new(0.31),
            },
        };
        obstacle_only_updates(noises, num_particles, &transcript, true);
    } else {
        println!("Usage: expr_obst0 transcript_filename num_particles");
    }
    Ok(())
}
