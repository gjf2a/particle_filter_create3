use particle_filter::{ParticleFilter, Radians, RobotPose};

use crate::{Noises, SensorInfo, circle_obstacles::CircleObstacles};

pub fn update_every_tick(noises: Noises, num_particles: usize, transcript: &Vec<SensorInfo>) {
    let starting_map = CircleObstacles::new(noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
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
}

pub fn obstacle_only_updates(
    noises: Noises,
    num_particles: usize,
    transcript: &Vec<SensorInfo>,
    print_each_update: bool,
) {
    let starting_map = CircleObstacles::new(noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    let mut raw = RobotPose::<Radians>::default();
    for (i, sensor_info) in transcript.iter().enumerate() {
        if let Some(odom) = sensor_info.odometry() {
            raw = odom;
        }
        if sensor_info.obstacles().is_some() {
            particle_filter.iterate(Some(raw), sensor_info.obstacles());
            particle_filter.iterate(Some(raw), None);
            let (est, _) = particle_filter.current_best();
            if print_each_update {
                println!("Updating; step {i}");
                println!("raw: {raw}");
                println!("est: {est}");
            }
        }
    }
    particle_filter.iterate(Some(raw), None);
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
}
