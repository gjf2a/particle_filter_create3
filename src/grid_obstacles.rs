use std::{f64::consts::PI, iter::repeat};

use bit_grid::BitGrid;
use particle_filter::{Angle, FloatPoint, Noise, ObstacleMap, Point, Radians, RobotPose};

use crate::{CREATE3_RADIUS, Noises, SensorInfo};

#[derive(Clone)]
pub struct GridObstacles {
    grid: BitGrid,
    square_size_m: f64,
    noises: Noises,
}

impl GridObstacles {
    pub fn new(square_size_m: f64, noises: Noises) -> Self {
        Self {grid: BitGrid::default(), square_size_m, noises}
    }

    fn grid_index<A: Angle>(&self, pose: &RobotPose<A>) -> Option<(i64, i64)> {
        let x = self.to_square(pose.pos[0]);
        let y = self.to_square(pose.pos[1]);
        if self.grid.in_bounds(x, y) {
            Some((x, y))
        } else {
            None
        }
    }
    
    fn to_square(&self, value_meters: f64) -> i64 {
        (value_meters / self.square_size_m) as i64
    }

    fn to_meters(&self, value_square: i64) -> f64 {
        value_square as f64 * self.square_size_m
    }

    fn to_float_point(&self, x: i64, y: i64) -> FloatPoint {
        FloatPoint::new([self.to_meters(x), self.to_meters(y)])
    }

    fn to_point(&self, fp: FloatPoint) -> Point<i64, 2> {
        fp.iter().map(|f| self.to_square(f)).collect()
    }

    fn robot_grid_radius(&self) -> i64 {
        self.to_square(CREATE3_RADIUS * 4.0 / PI)
    }

    pub fn robot_shadow(&self, pose: RobotPose<Radians>) -> BitGrid {
        let grid_point = self.to_point(pose.pos);
        let min = grid_point - repeat(self.robot_grid_radius()).collect::<Point<_,_>>();       
        let max = grid_point + repeat(self.robot_grid_radius()).collect::<Point<_,_>>();       
        let mut shadow = self.grid.zero_clone();
        for p in min.point_iter(&max) {
            if p.manhattan_distance(grid_point) <= self.robot_grid_radius() {
                shadow.set(p[0], p[1], true);
            }
        }
        shadow
    }
}



impl ObstacleMap for GridObstacles {
    type SensorType = SensorInfo;


    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        todo!()
    }
    
    fn error(&mut self, pose: RobotPose<Radians>) -> f64 {
        todo!()
    }
    
    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use particle_filter::{FloatPoint, Radians, RobotPose};

    use crate::{Noises, grid_obstacles::GridObstacles};

    #[test]
    fn test_shadow() {
        let mut tester = GridObstacles::new(0.1, Noises::default());
        let size = 3;
        tester.grid.set(size, size, false);
        tester.grid.set(-size, -size, false);
        let pose = RobotPose::<Radians> {pos: FloatPoint::new([0.0, 0.0]), theta: Radians::new(0.0)};
        let shadow = tester.robot_shadow(pose);
        let expected = "0000000
0001000
0011100
0111110
0011100
0001000
0000000";
        let shadow_str = format!("{shadow}");
        assert_eq!(expected, shadow_str); 
    }
}