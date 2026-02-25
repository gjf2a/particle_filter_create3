use particle_filter::{ObstacleMap, ParticleFilter, Radians, RobotPose, stats::Stats};

use crate::{
    Bump, Noises, circle_obstacles::CircleObstacles,
    grid_circle_obstacles::GridCircleObstacles, grid_obstacles::GridObstacles, odometry_transcripts::Transcript,
};

pub fn update_every_tick_circle(
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
) {
    let starting_map = CircleObstacles::new(noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    update_every_tick(&mut particle_filter, transcript);
    final_report(transcript, &particle_filter);
}

pub fn obstacle_only_updates_circle(
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
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
    transcript: &Transcript,
) {
    let starting_map = GridObstacles::new(square_size_m, noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    update_every_tick(&mut particle_filter, transcript);
    final_report(transcript, &particle_filter);
    let obstacle_count = particle_filter
        .particles()
        .map(|(_, m, _)| m.num_obstacles() as f64)
        .collect::<Stats<f64>>();
    println!(
        "Mean obstacles: [{}, {}] {:.2} (+/- {:.2})",
        obstacle_count.min(),
        obstacle_count.max(),
        obstacle_count.mean(),
        obstacle_count.stdev()
    );
    let (_, best_map, _) = particle_filter.current_best();
    println!("grid pixels: {}", best_map.height() * best_map.width());
}

pub fn update_every_tick_grid_circle(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
) {
    let starting_map = GridCircleObstacles::new(square_size_m, noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    update_every_tick(&mut particle_filter, transcript);
    final_report(transcript, &particle_filter);
}

pub fn obstacle_only_updates_grid(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
    print_each_update: bool,
) {
    let starting_map = GridObstacles::new(square_size_m, noises);
    let mut particle_filter = ParticleFilter::new(num_particles, &starting_map);
    obstacle_only_updates(&mut particle_filter, transcript, print_each_update);
    final_report(transcript, &particle_filter);
}

pub fn update_every_tick<M: ObstacleMap<SensorType = Bump>>(
    particle_filter: &mut ParticleFilter<M>,
    transcript: &Transcript,
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
    transcript: &Transcript,
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
            let (est, _, _) = particle_filter.current_best();
            if print_each_update {
                println!("Updating; step {i}");
                println!("raw: {raw}");
                println!("est: {est}");
            }
        }
    }
    particle_filter.iterate(Some(raw), None);
}

pub fn final_report<M: ObstacleMap>(
    transcript: &Transcript,
    particle_filter: &ParticleFilter<M>,
) {
    let (best_pose, best_map, _) = particle_filter.current_best();
    let odometry_pose = transcript.final_pose();
    println!("Odometry:      {odometry_pose}");
    println!("Actual:        {}", transcript.actual());
    println!("Error:         {}", transcript.error_robot_stop());
    println!("Best estimate: {best_pose}");
    println!("Error:         {}", transcript.error_to(best_pose.pos));
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
        .map(|(pose, map, _)| map.clone().error((*pose).into()))
        .collect()
}
