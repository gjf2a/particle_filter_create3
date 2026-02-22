use std::{f64::consts::PI, iter::repeat};

use bit_grid::BitGrid;
use particle_filter::{BoundingBox, FloatPoint, Noise, ObstacleMap, Point, Radians, RobotPose};

use crate::{Bump, CREATE3_RADIUS, Noises};

#[derive(Clone)]
pub struct GridCircleObstacles {
    grid: BitGrid,
    num_collisions: u64,
    square_size_m: f64,
    noises: Noises,
}

impl GridCircleObstacles {
    pub fn new(square_size_m: f64, noises: Noises) -> Self {
        Self {
            grid: BitGrid::default(),
            square_size_m,
            noises,
            num_collisions: 0,
        }
    }

    pub fn width(&self) -> i64 {
        self.grid.width()
    }

    pub fn height(&self) -> i64 {
        self.grid.height()
    }

    fn to_square(&self, value_meters: f64) -> i64 {
        (value_meters / self.square_size_m) as i64
    }

    fn to_meters(&self, value_squares: i64) -> f64 {
        value_squares as f64 * self.square_size_m
    }

    fn to_point(&self, fp: FloatPoint) -> Point<i64, 2> {
        fp.iter().map(|f| self.to_square(f)).collect()
    }

    fn robot_grid_radius(&self) -> i64 {
        self.to_square(CREATE3_RADIUS * 4.0 / PI)
    }

    pub fn num_obstacles(&self) -> u64 {
        self.grid.count_bits_on()
    }
}

impl ObstacleMap for GridCircleObstacles {
    type SensorType = Bump;

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }

    fn error(&mut self, pose: RobotPose<Radians>) -> f64 {
        let grid_pose = self.to_point(pose.pos);
        if let Some(obstacle) = self.grid.is_set(grid_pose[0], grid_pose[1]) {
            if obstacle {
                self.num_collisions += 1;
            }
        }
        self.num_collisions as f64
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        if let Some(bump) = sensor_info {
            let float_location = bump.bump_location(pose);
            let grid_point = self.to_point(float_location);
            let min = grid_point - repeat(self.robot_grid_radius()).collect::<Point<_, _>>();
            let max = grid_point + repeat(self.robot_grid_radius()).collect::<Point<_, _>>();
            for p in min.point_iter(&max) {
                if p.manhattan_distance(grid_point) <= self.robot_grid_radius() {
                    self.grid.set(p[0], p[1], true);
                }
            }
        }
    }

    fn bounding_box(&self) -> BoundingBox {
        let (min_x, max_x, min_y, max_y) = self.grid.x_min_x_max_y_min_y_max();
        [(min_x, min_y), (max_x, max_y)]
            .iter()
            .map(|(x, y)| FloatPoint::new([self.to_meters(*x), self.to_meters(*y)]))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use particle_filter::{FloatPoint, ObstacleMap, Radians, RobotPose};

    use crate::{Bump, CREATE3_RADIUS, Noises, grid_circle_obstacles::GridCircleObstacles};

    #[test]
    fn test_pixel_circle() {
        let mut tester = GridCircleObstacles::new(0.1, Noises::default());
        let bot = RobotPose {
            pos: FloatPoint::new([-CREATE3_RADIUS, 0.0]),
            theta: Radians::default(),
        };
        tester.sensor_update(bot, Some(&Bump::FrontCenter));
        let expected_str = "00100
01110
11111
01110
00100";
        assert_eq!(expected_str, format!("{}", tester.grid).as_str());
        assert_eq!((-2, 2, -2, 2), tester.grid.x_min_x_max_y_min_y_max());
    }
}
