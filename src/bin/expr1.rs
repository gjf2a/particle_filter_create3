// TODO:
//
// Run an initial experiment here.
// Develop a noise model using Jacob's data.
// Determine the target odometry by doing some more measurements.
// Then try some number of particles and see how it does.

use particle_filter::ParticleFilter;
use particle_filter_create3::{circle_obstacles::CircleObstacles, from_transcript};

fn main() -> anyhow::Result<()> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() == 3 {
        let transcript = from_transcript(args[1].as_str())?;
        let num_particles = args[2].parse::<usize>()?;
        let mut particle_filter = ParticleFilter::<CircleObstacles>::new(num_particles);
        for (i, sensor_info) in transcript.iter().enumerate() {
            if i % 1000 == 0 {
                println!("{i}/{}", transcript.len());
            }
            particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
        }
        let best_pose = particle_filter.current_best().0;
        let odometry_pose = transcript
            .iter()
            .rev()
            .find(|s| s.odometry().is_some())
            .map(|s| s.odometry().unwrap())
            .unwrap();
        println!("Odometry:      {odometry_pose}");
        println!("Best estimate: {best_pose}");
    } else {
        println!("Usage: expr1 transcript_filename num_particles");
    }
    Ok(())
}
