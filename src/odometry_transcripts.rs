use std::{
    fs::{self, File},
    io::{BufRead, BufReader},
};

use particle_filter::{FloatPoint, Radians, RobotPose};

use crate::SensorInfo;

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
