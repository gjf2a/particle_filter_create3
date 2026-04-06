use std::{
    fs::{self, File},
    io::{BufRead, BufReader},
};

const CREATE3_ODOMETRY_UPDATE_INTERVAL: f64 = 0.05;

use particle_filter::{
    angle::Radians,
    point::{BoundingBox, FloatPoint},
    pose::RobotPose,
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
    steps: Vec<SensorInfo>,
    actual_ending_point: FloatPoint,
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
        Ok(Self {
            steps: from_transcript(transcript_filename)?,
            actual_ending_point: parse_ending_point(actual_filename.as_str())?,
        })
    }

    pub fn actual(&self) -> FloatPoint {
        self.actual_ending_point
    }

    pub fn iter(&self) -> impl Iterator<Item = SensorInfo> {
        self.steps.iter().copied()
    }

    pub fn len(&self) -> usize {
        self.steps.len()
    }

    pub fn error_to(&self, target: FloatPoint) -> FloatPoint {
        self.actual_ending_point - target
    }

    pub fn final_pose(&self) -> RobotPose<Radians> {
        self.steps
            .iter()
            .rev()
            .find(|s| s.odometry().is_some())
            .map(|s| s.odometry().unwrap())
            .unwrap()
    }

    pub fn error_robot_stop(&self) -> FloatPoint {
        self.error_to(self.final_pose().pos)
    }

    pub fn total_distance_recorded(&self) -> f64 {
        let mut prev = FloatPoint::default();
        let mut total = 0.0;
        for info in self.steps.iter() {
            if let SensorInfo::Pose(robot_pose) = info {
                total += robot_pose.pos.euclidean_distance(prev);
                prev = robot_pose.pos;
            }
        }
        total
    }

    pub fn total_time_seconds(&self) -> f64 {
        self.steps.iter().filter(|s| s.odometry().is_some()).count() as f64
            * CREATE3_ODOMETRY_UPDATE_INTERVAL
    }

    pub fn bounding_box(&self) -> Option<BoundingBox<f64>> {
        self.steps
            .iter()
            .filter_map(|s| s.odometry())
            .map(|p| p.pos)
            .collect()
    }

    pub fn pose_to_actual(&self, pose: RobotPose<Radians>) -> PoseReport {
        PoseReport {
            pose,
            error: self.error_to(pose.pos),
            distance: self.actual().euclidean_distance(pose.pos),
        }
    }
}

fn from_transcript(transcript_filename: &str) -> anyhow::Result<Vec<SensorInfo>> {
    let file = BufReader::new(File::open(transcript_filename)?);
    file.lines().map(|line| Ok(line?.parse()?)).collect()
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
            assert_eq!(line_info, transcript[i]);
            let transcript_line = format!("{}", transcript[i]);
            assert_eq!(transcript_line, line);
        }
    }
}
