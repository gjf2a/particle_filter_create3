use std::{collections::HashMap, f64::consts::PI, iter::repeat};

use bit_grid::{BitGrid, FixedBitGrid};
use enum_iterator::{Sequence, all};
use hash_histogram::HashHistogram;
use particle_filter::{
    BoundingBox, FloatPoint, GridPoint, Noise, Point, Radians, RobotPose, SensorNoiseMap,
    consistent::{ConsistentMap, StatCollector},
};

use crate::{Bump, CREATE3_RADIUS, Noises};

#[derive(Copy, Clone, Eq, PartialEq, Hash, Sequence, Debug)]
pub enum Inconsistency {
    ObstacleSpaceOverlap,
    SeparatedSpaces,
    OffMap,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct GridBounds {
    width: u64,
    height: u64,
    square_size_m: f64,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl GridBounds {
    pub fn new(radius_border_multiplier: f64, bounds: &BoundingBox, square_size_m: f64) -> Self {
        let border = radius_border_multiplier * CREATE3_RADIUS;
        let width = (bounds.width() / square_size_m + border * 2.0) as u64;
        let height = (bounds.height() / square_size_m + border * 2.0) as u64;
        Self {
            width,
            height,
            square_size_m,
            min_x: bounds.min_x() - border,
            min_y: bounds.min_y() - border,
            max_x: bounds.max_x() + border,
            max_y: bounds.max_y() + border,
        }
    }

    pub fn in_bounds(&self, point: FloatPoint) -> bool {
        self.min_x <= point[0]
            && point[0] <= self.max_x
            && self.min_y <= point[1]
            && point[1] <= self.max_y
    }

    pub fn meters_to_square_size(&self, value_meters: f64) -> u64 {
        (value_meters / self.square_size_m) as u64
    }

    pub fn robot_grid_radius(&self) -> u64 {
        self.meters_to_square_size(CREATE3_RADIUS * 4.0 / PI)
    }

    pub fn grid_location(&self, p: FloatPoint) -> GridPoint {
        p.iter()
            .zip(self.min_meters().iter())
            .map(|(n, min)| self.meters_to_square_size(n - min))
            .collect()
    }

    pub fn blank_grid(&self) -> FixedBitGrid {
        FixedBitGrid::new(self.width, self.height)
    }

    pub fn width(&self) -> u64 {
        self.width
    }

    pub fn height(&self) -> u64 {
        self.height
    }

    pub fn max_point(&self) -> GridPoint {
        GridPoint::new([self.width - 1, self.height - 1])
    }

    pub fn min_meters(&self) -> FloatPoint {
        FloatPoint::new([self.min_x, self.min_y])
    }
}

pub struct RobotShadows {
    shadows: Vec<Vec<FixedBitGrid>>,
}

impl RobotShadows {
    pub fn new(bounds: &GridBounds) -> Self {
        let limit = bounds.max_point();
        let mut result = Self { shadows: vec![] };
        for x in 0..bounds.width() {
            let mut row = vec![];
            for y in 0..bounds.height() {
                let mut shadow = bounds.blank_grid();
                let grid_point = GridPoint::new([x, y]);
                let min = grid_point - repeat(bounds.robot_grid_radius()).collect::<Point<_, _>>();
                let max = grid_point + repeat(bounds.robot_grid_radius()).collect::<Point<_, _>>();
                let max = max.element_min(&limit);
                for p in min.point_iter(&max) {
                    if p.manhattan_distance(grid_point) <= bounds.robot_grid_radius() {
                        if shadow.in_bounds(p[0], p[1]) {
                            shadow.set(p[0], p[1], true);
                        } else {
                            panic!(
                                "Out of bounds: {p}; {limit}; {min} {max} {} {}",
                                bounds.width(),
                                bounds.height()
                            );
                        }
                    }
                }
                row.push(shadow);
            }
            result.shadows.push(row);
        }
        result
    }

    pub fn shadow(&self, x: u64, y: u64) -> &FixedBitGrid {
        let x = x as usize;
        let y = y as usize;
        &self.shadows[x][y]
    }
}

#[derive(Clone)]
pub struct FixedGridObstacles<'a> {
    bounds: GridBounds,
    obstacles: FixedBitGrid,
    spaces: FixedBitGrid,
    noises: Noises,
    brand_new: bool,
    space_contiguous: bool,
    out_of_bounds: bool,
    shadows: &'a RobotShadows,
}

impl<'a> FixedGridObstacles<'a> {
    pub fn new(bounds: GridBounds, noises: Noises, shadows: &'a RobotShadows) -> Self {
        Self {
            obstacles: bounds.blank_grid(),
            spaces: bounds.blank_grid(),
            bounds,
            noises,
            brand_new: true,
            space_contiguous: true,
            out_of_bounds: false,
            shadows,
        }
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
        } else if self.out_of_bounds {
            Some(Inconsistency::OffMap)
        } else {
            None
        }
    }
}

impl<'a> SensorNoiseMap for FixedGridObstacles<'a> {
    type SensorType = Bump;

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        if self.bounds.in_bounds(pose.pos) {
            match sensor_info {
                Some(bump) => {
                    let float_location = bump.bump_location(pose);
                    let p = self.bounds.grid_location(float_location);
                    if self.obstacles.in_bounds(p[0], p[1]) {
                        self.obstacles.set(p[0], p[1], true);
                    } else {
                        self.out_of_bounds = true;
                    }
                }
                None => {
                    let p = self.bounds.grid_location(pose.pos);
                    if self.spaces.in_bounds(p[0], p[1]) {
                        let shadow = self.shadows.shadow(p[0], p[1]);
                        let overlap = self.spaces.intersection(shadow).unwrap();
                        let overlap = overlap.count_bits_on() > 0;
                        self.space_contiguous =
                            self.space_contiguous && (self.brand_new || overlap);
                        self.brand_new = false;
                        self.spaces = self.spaces.union(shadow).unwrap();
                    } else {
                        self.out_of_bounds = true;
                    }
                }
            }
        } else {
            self.out_of_bounds = true;
        }
    }

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }
}

impl<'a> ConsistentMap for FixedGridObstacles<'a> {
    type StatType = FixedGridObstaclesStats;

    fn is_consistent(&self) -> bool {
        self.inconsistency().is_none()
    }
}

#[derive(Clone)]
pub struct FixedGridObstaclesStats {
    pub stats: HashMap<Inconsistency, HashHistogram<usize, usize>>,
}

impl FixedGridObstaclesStats {
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

impl Default for FixedGridObstaclesStats {
    fn default() -> Self {
        Self {
            stats: all::<Inconsistency>()
                .map(|inc| (inc, HashHistogram::default()))
                .collect(),
        }
    }
}

impl<'a> StatCollector<FixedGridObstacles<'a>> for FixedGridObstaclesStats {
    fn gather_data_from(&mut self, iteration: usize, particle: &FixedGridObstacles) {
        if let Some(inconsistency) = particle.inconsistency() {
            self.stats.get_mut(&inconsistency).unwrap().bump(&iteration);
        }
    }
}
