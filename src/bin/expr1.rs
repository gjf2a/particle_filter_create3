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
        let (best_pose, best_map) = particle_filter.current_best();
        let odometry_pose = transcript
            .iter()
            .rev()
            .find(|s| s.odometry().is_some())
            .map(|s| s.odometry().unwrap())
            .unwrap();
        println!("Odometry:      {odometry_pose}");
        println!("Best estimate: {best_pose}");
        println!("Bounding box:  {:?}", best_map.obstacle_extremes());
    } else {
        println!("Usage: expr1 transcript_filename num_particles");
    }
    Ok(())
}
