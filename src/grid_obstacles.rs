use std::{f64::consts::PI, iter::repeat};

use bit_grid::{BitGrid, GrowingBitGrid};
use eframe::egui::Color32;
use particle_filter::{
    BoundingBox, FloatPoint, Noise, ObstacleMap, Point, Radians, RobotPose, SensorNoiseMap,
    consistent::{ConsistentMap, StatCollector},
};

use crate::{
    Bump, CREATE3_RADIUS, Noises,
    fixed_grid_obstacles::{FixedGridObstaclesStats, Inconsistency},
};

#[derive(Copy, Clone)]
pub enum Cell {
    Obstacle,
    Space,
    Unvisited,
    Inconsistent,
}

impl Cell {
    pub fn color(&self) -> Color32 {
        match self {
            Cell::Obstacle => Color32::BLACK,
            Cell::Space => Color32::CYAN,
            Cell::Unvisited => Color32::KHAKI,
            Cell::Inconsistent => Color32::RED,
        }
    }
}

#[derive(Clone, PartialEq)]
pub struct GridObstacles {
    obstacles: GrowingBitGrid,
    spaces: GrowingBitGrid,
    square_size_m: f64,
    noises: Noises,
    brand_new: bool,
    space_contiguous: bool,
}

impl GridObstacles {
    pub fn new(square_size_m: f64, noises: Noises) -> Self {
        Self {
            obstacles: GrowingBitGrid::default(),
            spaces: GrowingBitGrid::default(),
            square_size_m,
            noises,
            brand_new: true,
            space_contiguous: true,
        }
    }

    pub fn upper_left_x_y(&self) -> (i64, i64) {
        let (x_min, _, y_min, _) = self.spaces.x_min_x_max_y_min_y_max();
        (x_min, y_min)
    }

    pub fn points(&self) -> impl Iterator<Item = (i64, i64, Cell)> {
        self.spaces.coord_iter().map(|(x, y)| {
            (
                x,
                y,
                if self.spaces.is_set(x, y) {
                    if self.obstacles.is_set(x, y) {
                        Cell::Inconsistent
                    } else {
                        Cell::Space
                    }
                } else if self.obstacles.is_set(x, y) {
                    Cell::Obstacle
                } else {
                    Cell::Unvisited
                },
            )
        })
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

    pub fn robot_shadow(&mut self, pose: RobotPose<Radians>) -> GrowingBitGrid {
        let mut shadow = self.obstacles.zero_clone();
        Self::draw_overlapping_shadow_on(
            self.robot_grid_radius(),
            self.to_point(pose.pos),
            &mut shadow,
        );
        shadow
    }

    pub fn draw_overlapping_shadow_on(
        robot_grid_radius: i64,
        grid_point: Point<i64, 2>,
        grid: &mut GrowingBitGrid,
    ) -> bool {
        let min = grid_point - repeat(robot_grid_radius).collect::<Point<_, _>>();
        let max = grid_point + repeat(robot_grid_radius).collect::<Point<_, _>>();
        let mut overlapping = false;
        for p in min.point_iter(&max) {
            if p.manhattan_distance(grid_point) <= robot_grid_radius {
                overlapping |= grid.is_set(p[0], p[1]);
                grid.set(p[0], p[1], true);
            }
        }
        overlapping
    }

    pub fn num_obstacles(&self) -> u64 {
        self.obstacles.count_bits_on()
    }

    pub fn space_contiguous(&self) -> bool {
        self.space_contiguous
    }

    pub fn obstacle_space_independent(&self) -> bool {
        let overlaps = self.obstacles.intersection(&self.spaces).unwrap();
        let osi = overlaps.ones().all(|(x, y)| {
            self.spaces
                .manhattan_neighbors(x, y)
                .any(|(_, _, is_on)| !is_on)
        });
        osi
    }

    pub fn inconsistency(&self) -> Option<Inconsistency> {
        if !self.space_contiguous {
            Some(Inconsistency::SeparatedSpaces)
        } else if !self.obstacle_space_independent() {
            Some(Inconsistency::ObstacleSpaceOverlap)
        } else {
            None
        }
    }
    /*
    pub fn frontier_spaces(&self) -> impl Iterator<Item = (i64,i64)> {
        let spaces_with_obstacles = self.spaces.union(&self.obstacles).unwrap();
        let frontier_spaces = spaces_with_obstacles.ones_touching_zeros().filter(|(x, y)| !self.obstacles.is_set(*x, *y));
        todo!();
    }

    pub fn filled_in(&self) -> bool {
        self.frontier_spaces().count() == 0
    }
    */
}

impl SensorNoiseMap for GridObstacles {
    type SensorType = Bump;

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        match sensor_info {
            Some(bump) => {
                let float_location = bump.bump_location(pose);
                let (x, y) = self.grid_index_unchecked(float_location);
                self.obstacles.set(x, y, true);
            }
            None => {
                let overlap = Self::draw_overlapping_shadow_on(
                    self.robot_grid_radius(),
                    self.to_point(pose.pos),
                    &mut self.spaces,
                );
                self.space_contiguous = self.space_contiguous && (self.brand_new || overlap);
                if !self.space_contiguous {
                    println!("space gap!");
                }
                self.brand_new = false;
            }
        }
        self.obstacles.match_sizes(&mut self.spaces);
    }
}

impl ObstacleMap for GridObstacles {
    type ErrorType = u64;

    fn error(&self) -> u64 {
        self.obstacles.overlapping_counts(&self.spaces).unwrap()
    }

    fn bounding_box(&self) -> BoundingBox {
        let (min_x, max_x, min_y, max_y) = self.obstacles.x_min_x_max_y_min_y_max();
        [(min_x, min_y), (max_x, max_y)]
            .iter()
            .map(|(x, y)| FloatPoint::new([self.to_meters(*x), self.to_meters(*y)]))
            .collect()
    }
}

impl ConsistentMap for GridObstacles {
    type StatType = FixedGridObstaclesStats;

    fn is_consistent(&self) -> bool {
        self.space_contiguous && self.obstacle_space_independent()
    }
}

impl StatCollector<GridObstacles> for FixedGridObstaclesStats {
    fn gather_data_from(&mut self, iteration: usize, particle: &GridObstacles) {
        if let Some(inconsistency) = particle.inconsistency() {
            self.stats.get_mut(&inconsistency).unwrap().bump(&iteration);
        }
    }
}

#[cfg(test)]
mod tests {
    use bit_grid::BitGrid;
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
