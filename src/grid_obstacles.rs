use std::collections::HashMap;

use bit_grid::{
    BitGrid,
    angle::Radians,
    point::{BoundingBox, FloatPoint, GridPoint, Point},
    pose::RobotPose,
    pt, span,
};
use eframe::egui::Color32;
use enum_iterator::{Sequence, all};
use hash_histogram::HashHistogram;
use particle_filter::{ConsistentMap, Noise, StatCollector};

use crate::{Bump, CREATE3_RADIUS, Noises};

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

fn to_square(square_size_m: f64, value_meters: f64) -> i64 {
    (value_meters / square_size_m) as i64
}

fn to_meters(square_size_m: f64, value_squares: i64) -> f64 {
    value_squares as f64 * square_size_m
}

fn to_float_point(square_size_m: f64, gp: GridPoint) -> FloatPoint {
    gp.iter().map(|g| to_meters(square_size_m, g)).collect()
}

fn to_grid_point(square_size_m: f64, fp: FloatPoint) -> GridPoint {
    fp.iter().map(|f| to_square(square_size_m, f)).collect()
}

#[derive(Clone, PartialEq)]
pub struct GridObstacles {
    obstacles: BitGrid,
    spaces: BitGrid,
    shadow: BitGrid,
    square_size_m: f64,
    noises: Noises,
    brand_new: bool,
    space_contiguous: bool,
}

impl GridObstacles {
    fn create_shadow(square_size_m: f64) -> BitGrid {
        let grid_radius = to_square(square_size_m, CREATE3_RADIUS);
        let mut shadow = BitGrid::new(-grid_radius, grid_radius, -grid_radius, grid_radius);
        for coord in shadow.coord_iter() {
            let float = to_float_point(square_size_m, coord);
            if float.euclidean_distance(pt!(0.0, 0.0)) < CREATE3_RADIUS {
                shadow.set(coord, true);
            }
        }
        shadow
    }

    pub fn new(square_size_m: f64, noises: Noises) -> Self {
        Self {
            obstacles: BitGrid::default(),
            spaces: BitGrid::default(),
            shadow: Self::create_shadow(square_size_m),
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

    pub fn points(&self) -> impl Iterator<Item = (GridPoint, Cell)> {
        self.spaces.coord_iter().map(|p| (p, self.cell_for(&p)))
    }

    pub fn cell_for(&self, p: &GridPoint) -> Cell {
        if self.spaces.get(p) {
            if self.obstacles.get(p) {
                if self.consistent_obstacle(p) {
                    Cell::Obstacle
                } else {
                    Cell::Inconsistent
                }
            } else {
                Cell::Space
            }
        } else if self.obstacles.get(p) {
            Cell::Obstacle
        } else {
            Cell::Unvisited
        }
    }

    pub fn bounding_box(&self) -> BoundingBox<i64> {
        self.spaces
            .bounding_box()
            .merge(self.obstacles.bounding_box())
    }

    pub fn width(&self) -> i64 {
        let bb = self.bounding_box();
        span(bb.min_x(), bb.max_x())
    }

    pub fn height(&self) -> i64 {
        let bb = self.bounding_box();
        span(bb.min_y(), bb.max_y())
    }

    fn grid_index_unchecked(&self, pos: FloatPoint) -> GridPoint {
        let x = self.to_square(pos[0]);
        let y = self.to_square(pos[1]);
        pt!(x, y)
    }

    fn to_square(&self, value_meters: f64) -> i64 {
        to_square(self.square_size_m, value_meters)
    }

    fn to_point(&self, fp: FloatPoint) -> GridPoint {
        to_grid_point(self.square_size_m, fp)
    }

    pub fn robot_shadow(&self, pose: RobotPose<Radians>) -> BitGrid {
        self.grid_shadow(self.to_point(pose.pos))
    }

    fn grid_shadow(&self, grid_point: GridPoint) -> BitGrid {
        self.shadow.translated(grid_point)
    }

    pub fn draw_overlapping_shadow_on(&mut self, grid_point: GridPoint) -> bool {
        let mut overlapping = false;
        for p in self.grid_shadow(grid_point).ones() {
            overlapping |= self.spaces.get(&p);
            self.spaces.set(p, true);
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
        self.obstacles.ones().all(|p| self.consistent_obstacle(&p))
    }

    pub fn num_neighbors_spaces(&self, p: &GridPoint) -> usize {
        self.spaces
            .manhattan_neighbors(p)
            .filter(|(_, is_on)| *is_on)
            .count()
    }

    pub fn consistent_obstacle(&self, p: &GridPoint) -> bool {
        let neighbor_spaces = self.num_neighbors_spaces(p);
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
        let spaces_with_obstacles = &self.spaces | &self.obstacles;
        spaces_with_obstacles
            .ones_touching_zeros()
            .filter(|p| !self.obstacles.get(p))
            .collect()
    }

    pub fn open_frontier_spaces(&self) -> BitGrid {
        self.all_frontier_spaces()
            .ones()
            .filter(|p| {
                let shadow = self.grid_shadow(*p);
                (&shadow & &self.obstacles).count_ones() == 0
            })
            .collect()
    }
}

impl ConsistentMap for GridObstacles {
    type SensorType = Bump;

    type StatType = GridObstaclesStats;

    fn is_consistent(&self) -> bool {
        self.space_contiguous && self.obstacle_space_independent()
    }

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        match sensor_info {
            Some(bump) => {
                let float_location = bump.bump_location(pose);
                let p = self.grid_index_unchecked(float_location);
                self.obstacles.set(p, true);
            }
            None => {
                let overlap = self.draw_overlapping_shadow_on(self.to_point(pose.pos));
                self.space_contiguous = self.space_contiguous && (self.brand_new || overlap);
                if !self.space_contiguous {
                    println!("space gap!");
                }
                self.brand_new = false;
            }
        }
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
    use bit_grid::{
        angle::Radians,
        point::{FloatPoint, Point},
        pose::RobotPose,
        pt,
    };

    use crate::{Noises, grid_obstacles::GridObstacles};

    #[test]
    fn test_shadow() {
        let mut tester = GridObstacles::new(0.1, Noises::default());
        let size = 3;
        tester.obstacles.set(pt!(size, size), false);
        tester.obstacles.set(pt!(-size, -size), false);
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

        tester.obstacles.set(pt!(0, 0), true);
        tester.obstacles.set(pt!(-2, -1), true);
        tester.obstacles.set(pt!(-2, 0), true);

        let intersected = tester.obstacles.overlapping_counts(&shadow);
        assert_eq!(intersected, 2);
    }
}
