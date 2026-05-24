use std::{
    fs::{self, File},
    io::{BufRead, BufReader},
    path::Path,
};

const CREATE3_ODOMETRY_UPDATE_INTERVAL: f64 = 0.05;

use particle_filter::{
    MapInput, ParticleFilter, angle::Radians, point::{BoundingBox, FloatPoint}, pose::RobotPose
};

use crate::SensorInfo;

#[derive(Clone)]
pub struct PoseReport {
    pose: RobotPose<Radians>,
    error: FloatPoint,
    distance: f64,
}

impl PoseReport {
    pub fn report(&self, name: &str) -> [String; 3] {
        [
            format!("{name} pose: {}", self.pose),
            format!(
                "{name} error: ({:.2}m, {:.2}m)",
                self.error[0], self.error[1]
            ),
            format!("{name} to actual: {:.2}m", self.distance),
        ]
    }

    pub fn distance(&self) -> f64 {
        self.distance
    }
}

#[derive(Clone)]
pub struct Transcript {
    steps: Vec<MapInput>,
    actual_ending_point: Option<FloatPoint>,
}

impl Transcript {
    pub fn from_transcript(transcript_filename: &str) -> anyhow::Result<Self> {
        let start = transcript_filename
            .find('_')
            .ok_or(anyhow::anyhow!("Bad format"))?;
        let end = transcript_filename
            .rfind('.')
            .ok_or(anyhow::anyhow!("wrong format"))?;
        let actual_filename = format!("actual{}", &transcript_filename[start..end]);
        let actual_ending_point = if Path::exists(&Path::new(&actual_filename)) {
            Some(parse_ending_point(actual_filename.as_str())?)
        } else {
            None
        };
        Ok(Self {
            steps: from_transcript(transcript_filename)?,
            actual_ending_point,
        })
    }
  
    pub fn from_particle_filter(particle_filter: &ParticleFilter) -> anyhow::Result<Self> {
        let map_inputs = particle_filter
            .inputs()
            .ok_or(anyhow::anyhow!("No inputs saved in particle filter"))?;
        Ok(Self {
            steps: map_inputs,
            actual_ending_point: particle_filter.get_actual_ending_point(),
        })
    }

    pub fn from_map_inputs(map_inputs: &Vec<MapInput>) -> Self {
        Self {
            steps: map_inputs.clone(),
            actual_ending_point: None,
        }
    }

    pub fn map_inputs(&self) -> &Vec<MapInput> {
        &self.steps
    }

    pub fn actual(&self) -> Option<FloatPoint> {
        self.actual_ending_point
    }

    pub fn iter(&self) -> impl Iterator<Item = MapInput> {
        self.steps.iter().copied()
    }

    pub fn len(&self) -> usize {
        self.steps.len()
    }

    pub fn error_to(&self, target: FloatPoint) -> Option<FloatPoint> {
        self.actual_ending_point.map(|end| end - target)
    }

    pub fn final_pose(&self) -> RobotPose<Radians> {
        self.steps
            .iter()
            .rev()
            .find(|s| s.pose().is_some())
            .map(|s| s.pose().unwrap())
            .unwrap()
    }

    pub fn error_robot_stop(&self) -> Option<FloatPoint> {
        self.error_to(self.final_pose().pos)
    }

    pub fn total_distance_recorded(&self) -> f64 {
        let mut prev = FloatPoint::default();
        let mut total = 0.0;
        for info in self.steps.iter() {
            if let Some(robot_pose) = info.pose() {
                total += robot_pose.pos.euclidean_distance(prev);
                prev = robot_pose.pos;
            }
        }
        total
    }

    pub fn total_time_seconds(&self) -> f64 {
        self.steps.iter().filter(|s| s.pose().is_some()).count() as f64
            * CREATE3_ODOMETRY_UPDATE_INTERVAL
    }

    pub fn bounding_box(&self) -> Option<BoundingBox<f64>> {
        self.steps
            .iter()
            .filter_map(|s| s.pose())
            .map(|p| p.pos)
            .collect()
    }

    pub fn pose_to_actual(&self, pose: RobotPose<Radians>) -> Option<PoseReport> {
        if let Some(error) = self.error_to(pose.pos) {
            if let Some(actual) = self.actual() {
                return Some(PoseReport {
                    pose,
                    error,
                    distance: actual.euclidean_distance(pose.pos),
                });
            }
        }
        None
    }
}

fn from_transcript(transcript_filename: &str) -> anyhow::Result<Vec<MapInput>> {
    let file = BufReader::new(File::open(transcript_filename)?);
    file.lines()
        .map(|line| Ok(line?.parse::<SensorInfo>()?.map_input()))
        .collect()
}

pub fn parse_ending_point(actual_filename: &str) -> anyhow::Result<FloatPoint> {
    let contents = fs::read_to_string(actual_filename)?;
    contents
        .lines()
        .map(|line| {
            let mut parts = line.split_whitespace().skip(1);
            let mut value = parts
                .next()
                .ok_or(anyhow::anyhow!("messed up"))?
                .parse::<f64>()?;
            match parts.next().ok_or(anyhow::anyhow!("Some other problem"))? {
                "cm" => {
                    value /= 100.0;
                }
                "m" => {}
                _ => anyhow::bail!("Bad units"),
            };
            match parts
                .next()
                .ok_or(anyhow::anyhow!("An additional problem"))?
            {
                "left" | "forward" | "ahead" => {}
                "right" | "backward" | "behind" => {
                    value = -value;
                }
                _ => anyhow::bail!("bad direction"),
            };
            Ok(value)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use std::{
        fs::File,
        io::{BufRead, BufReader},
    };

    use super::*;

    #[test]
    fn test_transcript() {
        let transcript = from_transcript("odometry_2026-02-12_10-49-47.out").unwrap();
        let file = BufReader::new(File::open("odometry_2026-02-12_10-49-47.out").unwrap());
        for (i, line) in file.lines().enumerate() {
            let line = line.unwrap();
            let line_info = line.parse::<SensorInfo>().unwrap();
            let map_input = line_info.map_input();
            assert_eq!(map_input, transcript[i]);
        }
    }
}
