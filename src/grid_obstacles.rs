use std::{f64::consts::PI, iter::repeat};

use bit_grid::BitGrid;
use particle_filter::{BoundingBox, FloatPoint, Noise, ObstacleMap, Point, Radians, RobotPose};

use crate::{Bump, CREATE3_RADIUS, Noises};

#[derive(Clone)]
pub struct GridObstacles {
    grid: BitGrid,
    square_size_m: f64,
    noises: Noises,
}

impl GridObstacles {
    pub fn new(square_size_m: f64, noises: Noises) -> Self {
        let mut result = Self {
            grid: BitGrid::default(),
            square_size_m,
            noises,
        };
        let shadow = result.robot_shadow(RobotPose::default());
        for (x, y, _) in shadow.iter() {
            result.grid.set(x, y, false);
        }
        result
    }

    fn grid_index_unchecked(&self, pos: FloatPoint) -> (i64, i64) {
        let x = self.to_square(pos[0]);
        let y = self.to_square(pos[1]);
        (x, y)
    }

    fn to_square(&self, value_meters: f64) -> i64 {
        (value_meters / self.square_size_m) as i64
    }

    fn to_point(&self, fp: FloatPoint) -> Point<i64, 2> {
        fp.iter().map(|f| self.to_square(f)).collect()
    }

    fn robot_grid_radius(&self) -> i64 {
        self.to_square(CREATE3_RADIUS * 4.0 / PI)
    }

    pub fn robot_shadow(&self, pose: RobotPose<Radians>) -> BitGrid {
        let grid_point = self.to_point(pose.pos);
        let min = grid_point - repeat(self.robot_grid_radius()).collect::<Point<_, _>>();
        let max = grid_point + repeat(self.robot_grid_radius()).collect::<Point<_, _>>();
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
    type SensorType = Bump;

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }

    fn error(&mut self, pose: RobotPose<Radians>) -> f64 {
        let shadow = self.robot_shadow(pose);
        println!("shadow: {:?}", shadow.x_min_x_max_y_min_y_max());
        println!("grid:   {:?}", self.grid.x_min_x_max_y_min_y_max());
        (self.grid.overlapping_counts(&shadow).unwrap()) as f64
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        if let Some(bump) = sensor_info {
            let float_location = bump.bump_location(pose);
            let (x, y) = self.grid_index_unchecked(float_location);
            self.grid.set(x, y, true);
        }
    }

    fn bounding_box(&self) -> BoundingBox {
        self.grid
            .iter()
            .filter(|(_, _, value)| *value)
            .map(|(x, y, _)| FloatPoint::new([x as f64, y as f64]))
            .collect()
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
        let pose = RobotPose::<Radians> {
            pos: FloatPoint::new([0.0, 0.0]),
            theta: Radians::new(0.0),
        };
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
        assert!(tester.grid.matching_dimensions(&shadow));
    }
}
