use std::cmp::Ordering;

use hash_histogram::HashHistogram;
use particle_filter::{
    FloatPoint, Radians, RobotPose, consistent::{ConsistentMap, ConsistentParticleFilter, SelectionStrategy},
};

use crate::{
    Noises, grid_obstacles::{GridObstacles, GridObstaclesStats, Inconsistency}, odometry_transcripts::Transcript
};

pub fn consistent_expr(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
) -> ConsistentParticleFilter<GridObstacles> {
    let starting_map = GridObstacles::new(square_size_m, noises);
    let mut particle_filter = ConsistentParticleFilter::new(
        num_particles,
        &starting_map,
        SelectionStrategy::DistanceWeight,
    );
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
    particle_filter
}

#[derive(Clone)]
pub struct SuccessData {
    pub closest_estimate: RobotPose<Radians>,
    pub closest_error: FloatPoint,
    pub estimate_to_actual: f64,
    pub map: GridObstacles,
    pub farthest_to_actual: f64,
}

impl SuccessData {
    pub fn all_and_open_frontier_counts(&self) -> (usize, usize, f64) {
        let all_counts = self.map.all_frontier_spaces().len();
        let open_counts = self.map.open_frontier_spaces().len();
        (
            all_counts,
            open_counts,
            open_counts as f64 / all_counts as f64,
        )
    }
}

#[derive(Clone)]
pub enum Estimate {
    Failure(usize),
    Success(SuccessData),
}

impl Estimate {
    pub fn new(
        transcript: &Transcript,
        particle_filter: &ConsistentParticleFilter<GridObstacles>,
    ) -> Self {
        if particle_filter.total_iterations() < transcript.len() {
            Self::Failure(particle_filter.total_iterations())
        } else {
            let (closest_estimate, estimate_to_actual, map) =
                closest_estimate(&transcript.actual(), &particle_filter);
            Self::Success(SuccessData {
                closest_estimate,
                closest_error: transcript.error_to(closest_estimate.pos),
                estimate_to_actual,
                map,
                farthest_to_actual: farthest_estimate(&transcript.actual(), particle_filter),
            })
        }
    }

    pub fn option(&self) -> Option<SuccessData> {
        match self {
            Estimate::Failure(_) => None,
            Estimate::Success(success_data) => Some(success_data.clone()),
        }
    }
}

#[derive(Clone)]
pub struct ConsistentData {
    pub outcome: Estimate,
    pub actual: FloatPoint,
    pub odometry_pose: RobotPose<Radians>,
    pub odometry_error: FloatPoint,
    pub odometry_distance: f64,
    pub iterations_with_inconsistencies: usize,
    pub total_inconsistencies: usize,
    pub obstacle_space_issues: usize,
    pub discontinuity_issues: usize,
    pub iteration_inconsistencies: HashHistogram<usize>,
}

impl ConsistentData {
    pub fn new(
        transcript: &Transcript,
        particle_filter: &ConsistentParticleFilter<GridObstacles>,
        stats: &GridObstaclesStats,
    ) -> Self {
        let iteration_inconsistencies = stats.by_iteration();
        Self {
            outcome: Estimate::new(transcript, particle_filter),
            actual: transcript.actual(),
            odometry_pose: transcript.final_pose(),
            odometry_error: transcript.error_robot_stop(),
            odometry_distance: transcript
                .error_robot_stop()
                .euclidean_distance(FloatPoint::new([0.0, 0.0])),
            iterations_with_inconsistencies: iteration_inconsistencies.len(),
            total_inconsistencies: iteration_inconsistencies.total_count(),
            obstacle_space_issues: stats.total_for(&Inconsistency::ObstacleSpaceOverlap),
            discontinuity_issues: stats.total_for(&Inconsistency::SeparatedSpaces),
            iteration_inconsistencies,
        }
    }

    pub fn print(&self) {
        println!("Actual position: {}", self.actual);
        println!("Odometry pose: {}", self.odometry_pose);
        println!("Odometry error: {}", self.odometry_error);
        println!("Odometry to actual: {:.2}", self.odometry_distance);
        match &self.outcome {
            Estimate::Failure(failure_iteration) => {
                println!("Failure Iteration: {failure_iteration}");
            }
            Estimate::Success(data) => {
                println!("Particle pose: {}", data.closest_estimate);
                println!("Particle error: {}", data.closest_error);
                println!("Particle to actual: {:.2}", data.estimate_to_actual);
                println!("Dimensions: {} x {}", data.map.width(), data.map.height());
                println!("Farthest particle distance: {}", data.farthest_to_actual);
            }
        }
        println!(
            "Iterations w/inconsistencies: {}",
            self.iterations_with_inconsistencies
        );
        println!("Total inconsistencies: {}", self.total_inconsistencies);
        println!("Total obstacle/space: {}", self.obstacle_space_issues);
        println!("Total discontinuity: {}", self.discontinuity_issues);
    }
}

pub fn closest_estimate<M: ConsistentMap>(
    actual: &FloatPoint,
    particles: &ConsistentParticleFilter<M>,
) -> (RobotPose<Radians>, f64, M) {
    particles
        .particles()
        .map(|p| {
            (
                p.estimated_pose(),
                p.estimated_pose().pos.euclidean_distance(*actual),
                p.map().clone(),
            )
        })
        .min_by(|(_, dist1, _), (_, dist2, _)| dist1.partial_cmp(dist2).unwrap_or(Ordering::Equal))
        .unwrap()
}

pub fn farthest_estimate<M: ConsistentMap>(
    actual: &FloatPoint,
    particles: &ConsistentParticleFilter<M>,
) -> f64 {
    particles
        .particles()
        .map(|p| p.estimated_pose().pos.euclidean_distance(*actual))
        .max_by(|dist1, dist2| dist1.partial_cmp(dist2).unwrap_or(Ordering::Equal))
        .unwrap()
}
