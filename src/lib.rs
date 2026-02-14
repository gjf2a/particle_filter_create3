use std::{fmt::Display, fs::File, io::{BufRead, BufReader}, str::FromStr};

use particle_filter::{RobotPose, point::FloatPoint};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SensorInfo {
    Pose(RobotPose),
    Bump(Bump),
}

pub fn from_transcript(transcript_filename: &str) -> anyhow::Result<Vec<SensorInfo>> {
    let file = BufReader::new(File::open(transcript_filename)?);
    file.lines().map(|line| Ok(line?.parse()?)).collect()
}

impl Display for SensorInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Pose(pose) => {
                write!(f, "{} {} {}", pose.pos[0], pose.pos[1], pose.theta)
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

impl FromStr for SensorInfo {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.chars().next() {
            None => Err(anyhow::anyhow!("Bad input: Empty input")),
            Some(c) => match c {
                '0'..='9' | '-' => Ok(Self::Pose(parse_pose(s.split_whitespace())?)),
                '[' => Ok(Self::Bump(parse_bump(s)?)),
                _ => Err(anyhow::anyhow!("Bad starting character: '{c}' in line '{s}'"))
            }
        }
    }
}

fn parse_pose<'a, I: Iterator<Item=&'a str>>(values: I) -> anyhow::Result<RobotPose> {
    let parts = values.map(|n| n.parse()).collect::<Vec<_>>();
    let mut values = vec![];
    for part in parts {
        match part {
            Ok(value) => {values.push(value)},
            Err(e) => {return Err(anyhow::anyhow!("{e}"))}
        }
    }
    if values.len() != 3 {
        return Err(anyhow::anyhow!("Need exactly 3 values, not {}", values.len()));
    } 
    Ok(RobotPose { pos: FloatPoint::new([values[0], values[1]]), theta: values[2] })
}

fn parse_bump(s: &str) -> anyhow::Result<Bump> {
    let start = s.find('\'').ok_or_else(|| anyhow::anyhow!("No starting '"))? + 1;
    let end = s.rfind('\'').ok_or_else(|| anyhow::anyhow!("No ending '"))?;
    let label = &s[start..end];
    match label {
        "bump_front_center" => Ok(Bump::FrontCenter),
        "bump_front_left" => Ok(Bump::FrontLeft),
        "bump_front_right" => Ok(Bump::FrontRight),
        "bump_left" => Ok(Bump::Left),
        "bump_right" => Ok(Bump::Right),
        _ => Err(anyhow::anyhow!("Did not recognize '{label}'"))
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

#[cfg(test)]
mod tests {
    use std::{
        fs::File,
        io::{BufRead, BufReader},
    };

    use super::*;

    #[test]
    fn bump_types() {
        let file = BufReader::new(File::open("odometry_2026-02-12_10-49-47.out").unwrap());
        let mut bumps = std::collections::BTreeSet::new();
        for line in file.lines() {
            let line = line.unwrap();
            if line.starts_with("[") {
                bumps.insert(line);
            }
        }
        println!("bump types");
        for bump in bumps {
            println!("{bump}");
        }
    }

    #[test]
    fn test_transcript() {
        let transcript = from_transcript("odometry_2026-02-12_10-49-47.out").unwrap();
        let file = BufReader::new(File::open("odometry_2026-02-12_10-49-47.out").unwrap());
        for (i, line) in file.lines().enumerate() {
            let line = line.unwrap();
            let line_info = line.parse::<SensorInfo>().unwrap();
            assert_eq!(line_info, transcript[i]);
        }
    }
}
