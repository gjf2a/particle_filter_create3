use particle_filter::{ObstacleMap, ParticleFilter, Radians, RobotPose, stats::Stats};

use crate::{
    Bump, Noises, SensorInfo, circle_obstacles::CircleObstacles, grid_obstacles::GridObstacles,
};

pub fn update_every_tick_circle(
    noises: Noises,
    num_particles: usize,
    transcript: &Vec<SensorInfo>,
) {
    let starting_map = CircleObstacles::new(noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    update_every_tick(&mut particle_filter, transcript);
    final_report(transcript, &particle_filter);
}

pub fn obstacle_only_updates_circle(
    noises: Noises,
    num_particles: usize,
    transcript: &Vec<SensorInfo>,
    print_each_update: bool,
) {
    let starting_map = CircleObstacles::new(noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    obstacle_only_updates(&mut particle_filter, transcript, print_each_update);
    final_report(transcript, &particle_filter);
}

pub fn update_every_tick_grid(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Vec<SensorInfo>,
) {
    let starting_map = GridObstacles::new(square_size_m, noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    update_every_tick(&mut particle_filter, transcript);
    final_report(transcript, &particle_filter);
}

pub fn obstacle_only_updates_grid(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Vec<SensorInfo>,
    print_each_update: bool,
) {
    let starting_map = GridObstacles::new(square_size_m, noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    obstacle_only_updates(&mut particle_filter, transcript, print_each_update);
    final_report(transcript, &particle_filter);
}

pub fn update_every_tick<M: ObstacleMap<SensorType = Bump>>(
    particle_filter: &mut ParticleFilter<M>,
    transcript: &Vec<SensorInfo>,
) {
    for (i, sensor_info) in transcript.iter().enumerate() {
        if i % 1000 == 0 {
            println!("{i}/{}", transcript.len());
        }
        particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
    }
}

pub fn obstacle_only_updates<M: ObstacleMap<SensorType = Bump>>(
    particle_filter: &mut ParticleFilter<M>,
    transcript: &Vec<SensorInfo>,
    print_each_update: bool,
) {
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
}

pub fn final_pose(transcript: &Vec<SensorInfo>) -> RobotPose<Radians> {
    transcript
        .iter()
        .rev()
        .find(|s| s.odometry().is_some())
        .map(|s| s.odometry().unwrap())
        .unwrap()
}

pub fn final_report<M: ObstacleMap>(
    transcript: &Vec<SensorInfo>,
    particle_filter: &ParticleFilter<M>,
) {
    let (best_pose, best_map) = particle_filter.current_best();
    let odometry_pose = final_pose(transcript);
    println!("Odometry:      {odometry_pose}");
    println!("Best estimate: {best_pose}");
    println!("Bounding box:  {:?}", best_map.bounding_box());
    let error_stats = error_stats(particle_filter);
    println!(
        "Mean error: {:.2} (+/- {:.2})",
        error_stats.mean(),
        error_stats.stdev()
    );
}

pub fn error_stats<M: ObstacleMap>(particle_filter: &ParticleFilter<M>) -> Stats<f64> {
    particle_filter
        .particles()
        .map(|(pose, map)| map.clone().error((*pose).into()))
        .collect()
}
