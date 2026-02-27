use std::{f64::consts::PI, iter::repeat};

use bit_grid::BitGrid;
use particle_filter::{BoundingBox, FloatPoint, Noise, ObstacleMap, Point, Radians, RobotPose};

use crate::{Bump, CREATE3_RADIUS, Noises};

#[derive(Clone, PartialEq)]
pub struct GridObstacles {
    obstacles: BitGrid,
    spaces: BitGrid,
    square_size_m: f64,
    noises: Noises,
}

impl GridObstacles {
    pub fn new(square_size_m: f64, noises: Noises) -> Self {
        Self {
            obstacles: BitGrid::default(),
            spaces: BitGrid::default(),
            square_size_m,
            noises,
        }
    }

    pub fn width(&self) -> i64 {
        self.obstacles.width()
    }

    pub fn height(&self) -> i64 {
        self.obstacles.height()
    }

    fn grid_index_unchecked(&self, pos: FloatPoint) -> (i64, i64) {
        let x = self.to_square(pos[0]);
        let y = self.to_square(pos[1]);
        (x, y)
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

    pub fn robot_shadow(&mut self, pose: RobotPose<Radians>) -> BitGrid {
        let mut shadow = self.obstacles.zero_clone();
        Self::draw_shadow_on(
            self.robot_grid_radius(),
            self.to_point(pose.pos),
            &mut shadow,
        );
        shadow
    }

    pub fn draw_shadow_on(robot_grid_radius: i64, grid_point: Point<i64, 2>, grid: &mut BitGrid) {
        let min = grid_point - repeat(robot_grid_radius).collect::<Point<_, _>>();
        let max = grid_point + repeat(robot_grid_radius).collect::<Point<_, _>>();
        for p in min.point_iter(&max) {
            if p.manhattan_distance(grid_point) <= robot_grid_radius {
                grid.set(p[0], p[1], true);
            }
        }
    }

    pub fn num_obstacles(&self) -> u64 {
        self.obstacles.count_bits_on()
    }
}

impl ObstacleMap for GridObstacles {
    type SensorType = Bump;
    type ErrorType = u64;

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }

    fn error(&self) -> u64 {
        self.obstacles.overlapping_counts(&self.spaces).unwrap()
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        match sensor_info {
            Some(bump) => {
                let float_location = bump.bump_location(pose);
                let (x, y) = self.grid_index_unchecked(float_location);
                self.obstacles.set(x, y, true);
            }
            None => {
                Self::draw_shadow_on(
                    self.robot_grid_radius(),
                    self.to_point(pose.pos),
                    &mut self.spaces,
                );
            }
        }
        self.obstacles.match_sizes(&mut self.spaces);
    }

    fn bounding_box(&self) -> BoundingBox {
        let (min_x, max_x, min_y, max_y) = self.obstacles.x_min_x_max_y_min_y_max();
        [(min_x, min_y), (max_x, max_y)]
            .iter()
            .map(|(x, y)| FloatPoint::new([self.to_meters(*x), self.to_meters(*y)]))
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
        tester.obstacles.set(size, size, false);
        tester.obstacles.set(-size, -size, false);
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
        assert!(tester.obstacles.matching_dimensions(&shadow));

        tester.obstacles.set(0, 0, true);
        tester.obstacles.set(-2, -1, true);
        tester.obstacles.set(-2, 0, true);

        let intersected = (tester.obstacles.overlapping_counts(&shadow)).unwrap();
        assert_eq!(intersected, 2);
    }
}
