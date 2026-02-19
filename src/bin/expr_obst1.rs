use particle_filter::{Degrees, Noise, ParticleFilter};
use particle_filter_create3::{
    circle_obstacles::{CircleObstacles, Noises},
    from_transcript,
};

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
        let starting_map = CircleObstacles::new(noises);
        let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
        for (i, sensor_info) in transcript.iter().enumerate() {
            if sensor_info.obstacles().is_some() {
                particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
                println!("Updating; step {i}");
            }
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
