use enum_iterator::all;
use particle_filter::{
    FloatPoint, ObstacleMap, ParticleFilter, Radians, RobotPose,
    consistent::{ConsistentMap, ConsistentParticleFilter},
    stats::Stats,
};
use std::cmp::Ordering;

use crate::{
    Bump, Noises,
    fixed_grid_obstacles::{FixedGridObstacles, GridBounds, Inconsistency, RobotShadows},
    grid_obstacles::GridObstacles,
    odometry_transcripts::Transcript,
};

pub fn fixed_consistent_driver(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
) {
    let bounds = GridBounds::new(&transcript.bounding_box(), square_size_m);
    let shadows = RobotShadows::new(&bounds);
    let starting_map = FixedGridObstacles::new(bounds, noises, &shadows);
    let mut particle_filter = ConsistentParticleFilter::new(num_particles, &starting_map);
    for (i, sensor_info) in transcript.iter().enumerate() {
        if i % 1000 == 0 {
            println!("{i}/{}", transcript.len());
        }
        particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
        if particle_filter.failed() {
            println!("Failed at iteration {i}");
            break;
        }
    }

    if !particle_filter.failed() {
        let odometry_pose = transcript.final_pose();
        println!("Odometry:      {odometry_pose}");
        println!("Actual:        {}", transcript.actual());
        println!("Error:         {}", transcript.error_robot_stop());
        let (closest, dist) = closest_estimate(&transcript.actual(), &particle_filter);
        println!("Estimate:      {closest} ({dist:.2})");
        println!("Error:         {}", transcript.error_to(closest.pos));
    }

    let stats = particle_filter.stats();
    for inc in all::<Inconsistency>() {
        let total = stats.stats_for(&inc);
        println!("Total {inc:?}: {}", total.total_count());
    }
}

pub fn consistent_driver(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
) {
    let starting_map = GridObstacles::new(square_size_m, noises);
    let mut particle_filter = ConsistentParticleFilter::new(num_particles, &starting_map);
    for (i, sensor_info) in transcript.iter().enumerate() {
        if i % 1000 == 0 {
            println!("{i}/{}", transcript.len());
        }
        particle_filter.iterate(sensor_info.odometry(), sensor_info.obstacles());
        if particle_filter.failed() {
            println!("Failed at iteration {i}");
            break;
        }
    }

    if !particle_filter.failed() {
        let odometry_pose = transcript.final_pose();
        println!("Odometry:      {odometry_pose}");
        println!("Actual:        {}", transcript.actual());
        println!("Error:         {}", transcript.error_robot_stop());
        let (closest, dist) = closest_estimate(&transcript.actual(), &particle_filter);
        println!("Estimate:      {closest} ({dist:.2})");
        println!("Error:         {}", transcript.error_to(closest.pos));
    }

    let stats = particle_filter.stats();
    let inconsistencies = stats.by_iteration();
    println!("Iterations with inconsistencies:  {}", inconsistencies.len());
    println!(
        "Total inconsistencies:            {}",
        inconsistencies.total_count()
    );
    println!("Total obstacle/space issues: {}", stats.total_collision());
    println!("Total discontinuity issues:  {}", stats.total_discontinuous());
    if particle_filter.failed() {
        println!("{inconsistencies}");
    }
}

pub fn closest_estimate<M: ConsistentMap>(
    actual: &FloatPoint,
    particles: &ConsistentParticleFilter<M>,
) -> (RobotPose<Radians>, f64) {
    particles
        .particles()
        .map(|p| {
            (
                p.estimated_pose(),
                p.estimated_pose().pos.euclidean_distance(*actual),
            )
        })
        .min_by(|(_, dist1), (_, dist2)| dist1.partial_cmp(dist2).unwrap_or(Ordering::Equal))
        .unwrap()
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
        .map(|p| p.map().num_obstacles() as f64)
        .collect::<Stats<f64>>();
    println!(
        "Mean obstacles: [{}, {}] {:.2} (+/- {:.2})",
        obstacle_count.min(),
        obstacle_count.max(),
        obstacle_count.mean(),
        obstacle_count.stdev()
    );
    let best_particle = particle_filter.current_best();
    let best_map = best_particle.map();
    println!("grid pixels: {}", best_map.height() * best_map.width());
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
            let est = particle_filter.current_best().estimated_pose();
            if print_each_update {
                println!("Updating; step {i}");
                println!("raw: {raw}");
                println!("est: {est}");
            }
        }
    }
    particle_filter.iterate(Some(raw), None);
}

pub fn final_report(transcript: &Transcript, particle_filter: &ParticleFilter<GridObstacles>) {
    let best_particle = particle_filter.current_best();
    let best_pose = best_particle.estimated_pose();
    let best_map = best_particle.map();
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
    println!(
        "Min, Max, Median error: {}, {}, {}",
        error_stats.min(),
        error_stats.median(),
        error_stats.max()
    );

    let range_stats = range_stats(particle_filter);
    println!(
        "Mean range to best: {:.2} (+/- {:.2})",
        range_stats.mean(),
        range_stats.stdev()
    );
    println!(
        "Min, Max, Median range: {}, {}, {}",
        range_stats.min(),
        range_stats.median(),
        range_stats.max()
    );
}
/*
pub fn error_stats<M: ObstacleMap>(particle_filter: &ParticleFilter<M>) -> Stats<f64> {
    particle_filter.particles().map(|p| p.error() as f64).collect()
}
*/

pub fn error_stats(particle_filter: &ParticleFilter<GridObstacles>) -> Stats<f64> {
    particle_filter
        .particles()
        .map(|p| p.error() as f64)
        .collect()
}

pub fn range_stats<M: ObstacleMap>(particle_filter: &ParticleFilter<M>) -> Stats<f64> {
    particle_filter
        .particles()
        .map(|p| {
            p.estimated_pose()
                .pos
                .euclidean_distance(particle_filter.current_best().estimated_pose().pos)
        })
        .collect()
}
