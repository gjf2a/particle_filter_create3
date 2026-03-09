use std::cmp::Ordering;

use bit_grid::{angle::Radians, point::FloatPoint, pose::RobotPose};
use hash_histogram::HashHistogram;
use particle_filter::{
    BitGridMap, BitGridStats, Inconsistency, Noises, ParticleFilter, SelectionStrategy,
};

use crate::{
    CREATE3_RADIUS,
    odometry_transcripts::{PoseReport, Transcript},
};

pub fn consistent_expr(
    square_size_m: f64,
    noises: Noises,
    num_particles: usize,
    transcript: &Transcript,
) -> ParticleFilter {
    let mut particle_filter = ParticleFilter::new(
        num_particles,
        square_size_m,
        CREATE3_RADIUS,
        noises,
        SelectionStrategy::RankProportion,
        particle_filter::WeightStrategy::MinPose,
    );
    for (i, sensor_info) in transcript.iter().enumerate() {
        if i % 1000 == 0 {
            println!("{i}/{}", transcript.len());
        }
        particle_filter.iterate(
            sensor_info.odometry(),
            sensor_info
                .obstacles()
                .map(|bump| bump.bump_location(&particle_filter.last_raw_pose().unwrap())),
        );
        if particle_filter.failed() {
            println!("Failed at iteration {i}");
            break;
        }
    }
    particle_filter
}

#[derive(Clone)]
pub struct SuccessData {
    pub best_particle_to_actual: PoseReport,
    pub closest_to_actual: PoseReport,
    pub closest_rank: usize,
    pub map: BitGridMap,
    pub farthest_to_actual: f64,
}

impl SuccessData {
    pub fn all_and_open_frontier_counts(&self) -> (usize, usize, f64) {
        let all_counts = self.map.all_frontier_spaces().count_ones();
        let open_counts = self.map.open_frontier_spaces().count_ones();
        (
            all_counts,
            open_counts,
            open_counts as f64 / all_counts as f64,
        )
    }
}

fn get_success_data(
    transcript: &Transcript,
    particle_filter: &ParticleFilter,
) -> Option<SuccessData> {
    if particle_filter.total_iterations() < transcript.len() {
        None
    } else {
        let best_pose = particle_filter.particles().next().unwrap().estimated_pose();
        let (closest_estimate, map, closest_rank) =
            closest_estimate(&transcript.actual(), &particle_filter);
        Some(SuccessData {
            best_particle_to_actual: transcript.pose_to_actual(best_pose),
            closest_to_actual: transcript.pose_to_actual(closest_estimate),
            closest_rank,
            map,
            farthest_to_actual: farthest_estimate(&transcript.actual(), particle_filter),
        })
    }
}

#[derive(Clone)]
pub struct ConsistentData {
    pub outcome: Option<SuccessData>,
    pub actual: FloatPoint,
    pub odometry_report: PoseReport,
    pub iterations_with_inconsistencies: usize,
    pub total_inconsistencies: usize,
    pub obstacle_space_issues: usize,
    pub discontinuity_issues: usize,
    pub iteration_inconsistencies: HashHistogram<usize>,
    pub particle_filter: ParticleFilter,
}

impl ConsistentData {
    pub fn new(
        transcript: &Transcript,
        particle_filter: &ParticleFilter,
        stats: &BitGridStats,
    ) -> Self {
        let iteration_inconsistencies = stats.by_iteration();
        Self {
            outcome: get_success_data(transcript, particle_filter),
            actual: transcript.actual(),
            odometry_report: transcript.pose_to_actual(transcript.final_pose()),
            iterations_with_inconsistencies: iteration_inconsistencies.len(),
            total_inconsistencies: iteration_inconsistencies.total_count(),
            obstacle_space_issues: stats.total_for(&Inconsistency::ObstacleSpaceOverlap),
            discontinuity_issues: stats.total_for(&Inconsistency::SeparatedSpaces),
            iteration_inconsistencies,
            particle_filter: particle_filter.clone(),
        }
    }

    pub fn print(&self) {
        println!("Actual position: {}", self.actual);
        for line in self.odometry_report.report("Odometry") {
            println!("{line}");
        }
        match &self.outcome {
            None => {
                println!("Failure");
            }
            Some(data) => {
                for line in data.best_particle_to_actual.report("Best-particle") {
                    println!("{line}");
                }
                for line in data.closest_to_actual.report("Closest-particle") {
                    println!("{line}");
                }
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

pub fn closest_estimate(
    actual: &FloatPoint,
    particles: &ParticleFilter,
) -> (RobotPose<Radians>, BitGridMap, usize) {
    particles
        .particles()
        .enumerate()
        .map(|(i, p)| {
            (
                p.estimated_pose(),
                p.estimated_pose().pos.euclidean_distance(*actual),
                p.map().clone(),
                i,
            )
        })
        .min_by(|(_, dist1, _, _), (_, dist2, _, _)| {
            dist1.partial_cmp(dist2).unwrap_or(Ordering::Equal)
        })
        .map(|(pose, _, map, i)| (pose, map, i + 1))
        .unwrap()
}

pub fn farthest_estimate(actual: &FloatPoint, particles: &ParticleFilter) -> f64 {
    particles
        .particles()
        .map(|p| p.estimated_pose().pos.euclidean_distance(*actual))
        .max_by(|dist1, dist2| dist1.partial_cmp(dist2).unwrap_or(Ordering::Equal))
        .unwrap()
}
