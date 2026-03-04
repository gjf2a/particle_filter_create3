use std::{collections::HashMap, f64::consts::PI, iter::repeat};

use bit_grid::BitGrid;
use eframe::egui::Color32;
use enum_iterator::{Sequence, all};
use hash_histogram::HashHistogram;
use particle_filter::{
    BoundingBox, FloatPoint, Noise, ObstacleMap, Point, Radians, RobotPose, SensorNoiseMap,
    consistent::{ConsistentMap, StatCollector},
};

use crate::{
    Bump, CREATE3_RADIUS, Noises,
};

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Cell {
    Obstacle,
    Space,
    Unvisited,
    Inconsistent,
}

impl Cell {
    pub fn color(&self) -> Color32 {
        match self {
            Cell::Obstacle => Color32::PURPLE,
            Cell::Space => Color32::LIGHT_BLUE,
            Cell::Unvisited => Color32::LIGHT_YELLOW,
            Cell::Inconsistent => Color32::RED,
        }
    }
}

#[derive(Clone, PartialEq)]
pub struct GridObstacles {
    obstacles: BitGrid,
    spaces: BitGrid,
    square_size_m: f64,
    noises: Noises,
    brand_new: bool,
    space_contiguous: bool,
}

impl GridObstacles {
    pub fn new(square_size_m: f64, noises: Noises) -> Self {
        Self {
            obstacles: BitGrid::default(),
            spaces: BitGrid::default(),
            square_size_m,
            noises,
            brand_new: true,
            space_contiguous: true,
        }
    }

    pub fn map_words_used(&self) -> u64 {
        self.obstacles.words_used() + self.spaces.words_used()
    }

    pub fn width_height_meters(&self) -> FloatPoint {
        FloatPoint::new([
            self.width() as f64 * self.square_size_m,
            self.height() as f64 * self.square_size_m,
        ])
    }

    pub fn upper_left_x_y(&self) -> (i64, i64) {
        let (x_min, _, y_min, _) = self.spaces.x_min_x_max_y_min_y_max();
        (x_min, y_min)
    }

    pub fn points(&self) -> impl Iterator<Item = (i64, i64, Cell)> {
        self.spaces
            .coord_iter()
            .map(|(x, y)| (x, y, self.cell_for(x, y)))
    }

    pub fn cell_for(&self, x: i64, y: i64) -> Cell {
        if self.spaces.is_set(x, y) {
            if self.obstacles.is_set(x, y) {
                if self.consistent_obstacle(x, y) {
                    Cell::Obstacle
                } else {
                    Cell::Inconsistent
                }
            } else {
                Cell::Space
            }
        } else if self.obstacles.is_set(x, y) {
            Cell::Obstacle
        } else {
            Cell::Unvisited
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

    pub fn robot_shadow(&self, pose: RobotPose<Radians>) -> BitGrid {
        self.grid_shadow(self.to_point(pose.pos))
    }

    fn grid_shadow(&self, grid_point: Point<i64, 2>) -> BitGrid {
        let mut shadow = self.obstacles.zero_clone();
        Self::draw_overlapping_shadow_on(self.robot_grid_radius(), grid_point, &mut shadow);
        shadow.downsize_to(&self.obstacles);
        shadow
    }

    pub fn draw_overlapping_shadow_on(
        robot_grid_radius: i64,
        grid_point: Point<i64, 2>,
        grid: &mut BitGrid,
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
        self.obstacles.count_ones()
    }

    pub fn num_spaces(&self) -> u64 {
        self.spaces.count_ones()
    }

    pub fn space_contiguous(&self) -> bool {
        self.space_contiguous
    }

    pub fn obstacle_space_independent(&self) -> bool {
        self.obstacles
            .ones()
            .all(|(x, y)| self.consistent_obstacle(x, y))
    }

    pub fn num_neighbors_spaces(&self, x: i64, y: i64) -> usize {
        self.spaces
            .manhattan_neighbors(x, y)
            .filter(|(_, _, is_on)| *is_on)
            .count()
    }

    pub fn consistent_obstacle(&self, x: i64, y: i64) -> bool {
        let neighbor_spaces = self.num_neighbors_spaces(x, y);
        0 < neighbor_spaces && neighbor_spaces < 4
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

    pub fn all_frontier_spaces(&self) -> BitGrid {
        let spaces_with_obstacles = self.spaces.union(&self.obstacles).unwrap();
        spaces_with_obstacles
            .ones_touching_zeros()
            .filter(|(x, y)| !self.obstacles.is_set(*x, *y))
            .collect()
    }

    pub fn open_frontier_spaces(&self) -> BitGrid {
        self.all_frontier_spaces()
            .ones()
            .filter(|(x, y)| {
                let shadow = self.grid_shadow(Point::<i64, 2>::new([*x, *y]));
                assert!(shadow.matching_dimensions(&self.obstacles));
                shadow
                    .intersection(&self.obstacles)
                    .unwrap()
                    .count_ones()
                    == 0
            })
            .collect()
    }
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
    type StatType = GridObstaclesStats;

    fn is_consistent(&self) -> bool {
        self.space_contiguous && self.obstacle_space_independent()
    }
}

impl StatCollector<GridObstacles> for GridObstaclesStats {
    fn gather_data_from(&mut self, iteration: usize, particle: &GridObstacles) {
        if let Some(inconsistency) = particle.inconsistency() {
            self.stats.get_mut(&inconsistency).unwrap().bump(&iteration);
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Sequence, Debug)]
pub enum Inconsistency {
    ObstacleSpaceOverlap,
    SeparatedSpaces,
    OffMap,
}

#[derive(Clone)]
pub struct GridObstaclesStats {
    pub stats: HashMap<Inconsistency, HashHistogram<usize, usize>>,
}

impl GridObstaclesStats {
    pub fn stats_for(&self, key: &Inconsistency) -> &HashHistogram<usize, usize> {
        self.stats.get(key).unwrap()
    }

    pub fn total_for(&self, key: &Inconsistency) -> usize {
        self.stats_for(key).total_count()
    }

    pub fn total(&self) -> usize {
        all::<Inconsistency>().map(|inc| self.total_for(&inc)).sum()
    }

    pub fn by_iteration(&self) -> HashHistogram<usize, usize> {
        let mut result = HashHistogram::new();
        for counts in self.stats.values() {
            for (key, count) in counts.iter() {
                result.bump_by(key, *count);
            }
        }
        result
    }
}

impl Default for GridObstaclesStats {
    fn default() -> Self {
        Self {
            stats: all::<Inconsistency>()
                .map(|inc| (inc, HashHistogram::default()))
                .collect(),
        }
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
