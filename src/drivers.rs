use std::cmp::Ordering;

use bit_grid::{angle::Radians, point::FloatPoint, pose::RobotPose};
use hash_histogram::HashHistogram;
use particle_filter::{
    BitGridMap, BitGridStats, Inconsistency, ParticleFilter, ParticleFilterSettings,
};
use stats_ci::{Confidence, proportion};

use crate::odometry_transcripts::{PoseReport, Transcript};

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
pub struct OneRunData {
    pub outcome: Option<SuccessData>,
    pub actual: FloatPoint,
    pub odometry_report: PoseReport,
    pub final_iteration: usize,
    pub transcript_len: usize,
    pub duration: f64,
    pub mean_iteration_time: f64,
    pub max_iteration_time: f64,
    pub iterations_with_inconsistencies: usize,
    pub total_inconsistencies: usize,
    pub obstacle_space_issues: usize,
    pub discontinuity_issues: usize,
    pub iteration_inconsistencies: HashHistogram<usize>,
    pub particle_filter: ParticleFilter,
}

impl OneRunData {
    pub fn new(
        transcript: &Transcript,
        particle_filter: &ParticleFilter,
        stats: &BitGridStats,
        duration: f64,
        mean_iteration_time: f64,
        max_iteration_time: f64,
        final_iteration: usize,
    ) -> Self {
        let iteration_inconsistencies = stats.by_iteration();
        Self {
            outcome: get_success_data(transcript, particle_filter),
            actual: transcript.actual(),
            odometry_report: transcript.pose_to_actual(transcript.final_pose()),
            duration,
            mean_iteration_time,
            max_iteration_time,
            iterations_with_inconsistencies: iteration_inconsistencies.len(),
            total_inconsistencies: iteration_inconsistencies.total_count(),
            obstacle_space_issues: stats.total_for(&Inconsistency::ObstacleSpaceOverlap),
            discontinuity_issues: stats.total_for(&Inconsistency::SeparatedSpaces),
            iteration_inconsistencies,
            particle_filter: particle_filter.clone(),
            final_iteration,
            transcript_len: transcript.len(),
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

pub struct MultiRunData {
    pub data: Vec<OneRunData>,
    pub settings: ParticleFilterSettings,
}

impl MultiRunData {
    pub fn num_runs(&self) -> usize {
        self.data.len()
    }

    pub fn num_successes(&self) -> usize {
        self.data.iter().filter(|d| d.outcome.is_some()).count()
    }

    pub fn num_failures(&self) -> usize {
        self.num_runs() - self.num_successes()
    }

    pub fn confidence_interval_success(&self) -> anyhow::Result<ConfIntervalConsistency> {
        ConfIntervalConsistency::new(0.95, self.data.len(), self.num_successes())
    }

    pub fn to_csv(&self) -> String {
        let mut csv = String::new();
        csv.push_str("num_particles,robot_radius_m,square_size_m,selection_strategy,weight_strategy,clear_x_y_noise,clear_theta_noise,collide_x_y_noise,collide_theta_noise\n");
        csv.push_str(&format!(
            "{},{},{},{:?},{:?},{},{},{},{}\n\n",
            self.settings.num_particles,
            self.settings.robot_radius_m,
            self.settings.square_size_m,
            self.settings.selection_strategy,
            self.settings.weight_strategy,
            self.settings.noises.clear.stdev_x_y,
            self.settings.noises.clear.stdev_angle,
            self.settings.noises.obst.stdev_x_y,
            self.settings.noises.obst.stdev_angle
        ));
        csv.push_str("succeeds,num_iterations,transcript_length,duration,mean_iteration_time,max_iteration_time,closest_particle_rank,closest_particle_to_actual,best_particle_to_actual,farthest_particle_to_actual,odometry_to_actual,open_frontier,all_frontier,all_space,num_obstacles,map_area\n");
        for row in self.data.iter() {
            csv.push_str(&match row.outcome.as_ref() {
                None => format!(
                    "0,{},{},{},{},{}\n",
                    row.final_iteration,
                    row.transcript_len,
                    row.duration,
                    row.mean_iteration_time,
                    row.max_iteration_time
                ),
                Some(outcome) => format!(
                    "1,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
                    row.final_iteration,
                    row.transcript_len,
                    row.duration,
                    row.mean_iteration_time,
                    row.max_iteration_time,
                    outcome.closest_rank,
                    outcome.closest_to_actual.distance(),
                    outcome.best_particle_to_actual.distance(),
                    outcome.farthest_to_actual,
                    row.odometry_report.distance(),
                    outcome.map.open_frontier_spaces().count_ones(),
                    outcome.map.all_frontier_spaces().count_ones(),
                    outcome.map.num_spaces(),
                    outcome.map.num_obstacles(),
                    outcome.map.area(),
                ),
            });
        }
        csv.push_str(",,,,,,,,,,,,,,,\n");
        csv.push_str(&format!("{},,,,,,,,,,,,,,,\n", self.num_successes()));
        if let Ok(ci) = self.confidence_interval_success() {
            csv.push_str(&format!("{},{},,,,,,,,,,,,,,\n", ci.lo_f, ci.lo));
            csv.push_str(&format!("{},{},,,,,,,,,,,,,,\n", ci.hi_f, ci.hi));
        }
        csv
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ConfIntervalConsistency {
    pub lo: usize,
    pub hi: usize,
    pub lo_f: f64,
    pub hi_f: f64,
}

impl ConfIntervalConsistency {
    pub fn new(confidence: f64, population: usize, successes: usize) -> anyhow::Result<Self> {
        let confidence = Confidence::new(confidence);
        let interval = proportion::ci(confidence, population, successes)?;
        let lo_f = interval.low_f();
        let hi_f = interval.high_f();
        let lo = (population as f64 * lo_f) as usize;
        let hi = (population as f64 * hi_f) as usize;
        Ok(Self { lo, hi, lo_f, hi_f })
    }
}
