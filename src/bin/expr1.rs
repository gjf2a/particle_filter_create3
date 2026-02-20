use particle_filter::{Degrees, Noise};
use particle_filter_create3::{
    Noises, drivers::update_every_tick, from_transcript
};

fn main() -> anyhow::Result<()> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() == 3 {
        let transcript = from_transcript(args[1].as_str())?;
        let num_particles = args[2].parse::<usize>()?;
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
        update_every_tick(noises, num_particles, &transcript);
    } else {
        println!("Usage: expr1 transcript_filename num_particles");
    }
    Ok(())
}
