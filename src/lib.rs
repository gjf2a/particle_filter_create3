pub mod drivers;
pub mod odometry_transcripts;

use std::{fmt::Display, str::FromStr};

use eframe::egui::Color32;
use particle_filter::{Cell, MapInput, irobot_create3::Bump};
use particle_filter::{angle::Radians, point::FloatPoint, pose::RobotPose};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SensorInfo {
    Pose(RobotPose<Radians>),
    Bump(Bump),
}

impl SensorInfo {
    pub fn odometry(&self) -> Option<RobotPose<Radians>> {
        match self {
            Self::Bump(_) => None,
            Self::Pose(pose) => Some(*pose),
        }
    }

    pub fn obstacles(&self) -> Option<&Bump> {
        match self {
            Self::Pose(_) => None,
            Self::Bump(bump) => Some(bump),
        }
    }

    pub fn map_input(&self) -> MapInput {
        match self {
            Self::Pose(pose) => MapInput::Pose(*pose),
            Self::Bump(bump) => {
                let (distance, heading) = bump.obstacle_at();
                MapInput::Obstacle(distance, heading)
            }
        }
    }
}

impl Display for SensorInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Pose(pose) => {
                write!(
                    f,
                    "{} {} {}",
                    f2py(pose.pos[0]),
                    f2py(pose.pos[1]),
                    f2py(pose.theta.into())
                )
            }
            Self::Bump(bump) => {
                let bump_str = match bump {
                    Bump::FrontCenter => "front_center",
                    Bump::FrontLeft => "front_left",
                    Bump::FrontRight => "front_right",
                    Bump::Left => "left",
                    Bump::Right => "right",
                };
                write!(f, "['bump_{bump_str}']")
            }
        }
    }
}

// From Perplexity
fn f64_dec_exp(x: f64) -> i32 {
    if x == 0.0 {
        return 0;
    }
    x.abs().log10().floor() as i32
}

fn f2py(f: f64) -> String {
    let mut buffer = ryu::Buffer::new();
    let mut s = buffer.format(f).to_string();
    if !s.contains(".") {
        s = format!("{s}.0");
    }
    if s.contains("e") && !s.contains("e-") {
        s = s.replace("e", "e+");
    }
    if f64_dec_exp(f) == -5 {
        s = format!("{f:e}");
    }
    for i in 1..=9 {
        let negexp = format!("e-{i}");
        if s.ends_with(&negexp) {
            let pynegexp = format!("e-0{i}");
            s = s.replace(&negexp, &pynegexp);
        }
    }
    s
}

impl FromStr for SensorInfo {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> anyhow::Result<Self> {
        match s.chars().next() {
            None => Err(anyhow::anyhow!("Bad input: Empty input")),
            Some(c) => match c {
                '0'..='9' | '-' => Ok(Self::Pose(parse_pose(s.split_whitespace())?)),
                '[' => Ok(Self::Bump(s.parse::<Bump>()?)),
                _ => Err(anyhow::anyhow!(
                    "Bad starting character: '{c}' in line '{s}'"
                )),
            },
        }
    }
}

fn parse_pose<'a, I: Iterator<Item = &'a str>>(values: I) -> anyhow::Result<RobotPose<Radians>> {
    let parts = values.map(|n| n.parse()).collect::<Vec<_>>();
    let mut values = vec![];
    for part in parts {
        match part {
            Ok(value) => values.push(value),
            Err(e) => return Err(anyhow::anyhow!("{e}")),
        }
    }
    if values.len() != 3 {
        return Err(anyhow::anyhow!(
            "Need exactly 3 values, not {}",
            values.len()
        ));
    }
    Ok(RobotPose {
        pos: FloatPoint::new([values[0], values[1]]),
        theta: Radians::new(values[2]),
    })
}

pub fn cell2color(cell: &Cell) -> Color32 {
    match cell {
        Cell::Obstacle => Color32::PURPLE,
        Cell::Space => Color32::LIGHT_BLUE,
        Cell::Unvisited => Color32::LIGHT_YELLOW,
        Cell::Inconsistent => Color32::RED,
    }
}
