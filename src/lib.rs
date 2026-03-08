pub mod drivers;
pub mod odometry_transcripts;

use std::{f64::consts::PI, fmt::Display, str::FromStr};

use bit_grid::{angle::Radians, point::FloatPoint, pose::RobotPose};
use eframe::egui::Color32;
use particle_filter::Cell;

pub const CREATE3_RADIUS: f64 = 0.2032; // meters

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
        if s.ends_with(negexp.as_str()) {
            let pynegexp = format!("e-0{i}");
            s = s.replace(negexp.as_str(), pynegexp.as_str());
        }
    }
    s
}

impl FromStr for SensorInfo {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.chars().next() {
            None => Err(anyhow::anyhow!("Bad input: Empty input")),
            Some(c) => match c {
                '0'..='9' | '-' => Ok(Self::Pose(parse_pose(s.split_whitespace())?)),
                '[' => Ok(Self::Bump(parse_bump(s)?)),
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

fn parse_bump(s: &str) -> anyhow::Result<Bump> {
    let start = s
        .find('\'')
        .ok_or_else(|| anyhow::anyhow!("No starting '"))?
        + 1;
    let end = s
        .rfind('\'')
        .ok_or_else(|| anyhow::anyhow!("No ending '"))?;
    let label = &s[start..end];
    match label {
        "bump_front_center" | "cliff_front_center" | "cliff_front_left', 'cliff_front_right" => {
            Ok(Bump::FrontCenter)
        }
        "bump_front_left" | "cliff_front_left" | "cliff_side_left', 'cliff_front_left" => {
            Ok(Bump::FrontLeft)
        }
        "bump_front_right" | "cliff_front_right" => Ok(Bump::FrontRight),
        "bump_left" | "cliff_side_left" => Ok(Bump::Left),
        "bump_right" | "cliff_side_right" => Ok(Bump::Right),
        _ => Err(anyhow::anyhow!("Did not recognize '{label}'")),
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Bump {
    FrontCenter,
    FrontLeft,
    FrontRight,
    Left,
    Right,
}

impl Bump {
    pub fn angle_offset(&self) -> Radians {
        match self {
            Bump::FrontCenter => Radians::new(0.0),
            Bump::FrontLeft => Radians::new(PI / 4.0),
            Bump::FrontRight => Radians::new(-PI / 4.0),
            Bump::Left => Radians::new(PI / 2.0),
            Bump::Right => Radians::new(-PI / 2.0),
        }
    }

    pub fn bump_location(&self, pose: &RobotPose<Radians>) -> FloatPoint {
        let heading = pose.theta + self.angle_offset();
        pose.pos + (CREATE3_RADIUS, heading).into()
    }
}

pub fn cell2color(cell: &Cell) -> Color32 {
    match cell {
        Cell::Obstacle => Color32::PURPLE,
        Cell::Space => Color32::LIGHT_BLUE,
        Cell::Unvisited => Color32::LIGHT_YELLOW,
        Cell::Inconsistent => Color32::RED,
    }
}
