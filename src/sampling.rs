use ilattice3::{Extent, Point};
use rand::{prelude::*, rngs::SmallRng};
use rand_distr::{Distribution, Normal, Uniform};
use serde::{Deserialize, Serialize};
use std::mem;

pub fn small_rng(seed: [u32; 4]) -> SmallRng {
    SmallRng::from_seed(unsafe { mem::transmute(seed) })
}

#[derive(Clone, Default, Deserialize, Serialize)]
pub struct NormalDistSpec {
    pub mean: f32,
    pub std_dev: f32,
}

impl NormalDistSpec {
    pub fn make(&self) -> Normal<f32> {
        Normal::new(self.mean, self.std_dev).expect("Bad parameters for normal distribution")
    }
}

pub struct LatticeNormalDist {
    pub x: Normal<f32>,
    pub y: Normal<f32>,
    pub z: Normal<f32>,
}

impl Distribution<Point> for LatticeNormalDist {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Point {
        let x = rng.sample(self.x).round() as i32;
        let y = rng.sample(self.y).round() as i32;
        let z = rng.sample(self.z).round() as i32;

        [x, y, z].into()
    }
}

#[derive(Clone, Default, Deserialize, Serialize)]
pub struct LatticeNormalDistSpec {
    pub x: NormalDistSpec,
    pub y: NormalDistSpec,
    pub z: NormalDistSpec,
}

impl LatticeNormalDistSpec {
    pub fn make(&self) -> LatticeNormalDist {
        LatticeNormalDist {
            x: self.x.make(),
            y: self.y.make(),
            z: self.z.make(),
        }
    }
}

pub struct LatticeUniformDist {
    pub x: Uniform<i32>,
    pub y: Uniform<i32>,
    pub z: Uniform<i32>,
}

impl Distribution<Point> for LatticeUniformDist {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Point {
        let x = rng.sample(self.x);
        let y = rng.sample(self.y);
        let z = rng.sample(self.z);

        [x, y, z].into()
    }
}

#[derive(Clone, Default, Deserialize, Serialize)]
pub struct LatticeUniformDistSpec {
    pub x: (i32, i32),
    pub y: (i32, i32),
    pub z: (i32, i32),
}

impl LatticeUniformDistSpec {
    pub fn make(&self) -> LatticeUniformDist {
        LatticeUniformDist {
            x: Uniform::new_inclusive(self.x.0, self.x.1),
            y: Uniform::new_inclusive(self.y.0, self.y.1),
            z: Uniform::new_inclusive(self.z.0, self.z.1),
        }
    }
}

/// Returns a random subrange of `[min, max]`,
pub fn sample_range<R: Rng>(rng: &mut R, min: i32, max: i32) -> (i32, i32) {
    let dist = Uniform::from(min..=max);
    let c1 = dist.sample(rng);
    let c2 = dist.sample(rng);

    if c1 > c2 {
        (c2, c1)
    } else {
        (c1, c2)
    }
}

pub fn sample_extents(
    num_extents: usize,
    predicate: impl Fn(&Extent) -> bool,
    location_distr: impl Distribution<Point>,
    size_distr: impl Distribution<Point>,
    rng: &mut impl Rng,
) -> Vec<Extent> {
    let mut extents = Vec::new();
    while extents.len() < num_extents {
        let loc = location_distr.sample(rng);
        let size = size_distr.sample(rng);
        let extent = Extent::from_min_and_local_supremum(loc, size);
        if predicate(&extent) {
            extents.push(extent);
        }
    }

    extents
}
