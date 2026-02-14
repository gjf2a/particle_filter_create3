use bits::BitArray;
use particle_filter::{Particle, RobotPose, point::{FloatPoint, GridPoint}};
use crate::SensorInfo;

#[derive(Clone)]
pub struct Grid {
    pose: RobotPose,
    obstacles: BitArray,
    meters_per_square: f64,
    lower_left: FloatPoint,
    upper_right: FloatPoint,
    dim: GridPoint,
}

impl Grid {
    pub fn new(meters_per_square: f64) -> Self {
        let dim: GridPoint = 1.into();
        Self {
            pose: RobotPose::default(),
            obstacles: BitArray::zeros(dim.iter().product()),
            meters_per_square,
            dim,
            lower_left: FloatPoint::default(),
            upper_right: FloatPoint::default(),
        }
    }

    pub fn grid_origin(&self) -> GridPoint {
        let mean = (self.lower_left + self.upper_right) / 2.0;
        mean.iter().zip(self.lower_left.iter()).map(|(m, l)| (m * self.meters_per_square - l) as u64).collect()
    }
}

impl Particle for Grid {
    type SensorType = SensorInfo;

    fn pose(&self) -> RobotPose {
        self.pose
    }
    
    fn error(&self) -> f64 {
        todo!()
    }
    
    fn update<N: Fn(RobotPose, &SensorInfo) -> RobotPose>(&mut self, sensor_info: &SensorInfo, noise_func: N) {
        match sensor_info {
            SensorInfo::Pose(p) => {
                self.pose = noise_func(*p, sensor_info);
                todo!("Find out square. Occupy it. Expand map if needed")
            }
            SensorInfo::Bump(b) => {
                
                todo!("Add obstacle to map")
            }
        }
    }
}