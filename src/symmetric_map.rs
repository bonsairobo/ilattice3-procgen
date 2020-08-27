use fnv::FnvHashMap;

#[derive(Default)]
pub struct SymmetricMap<T> {
    map: FnvHashMap<(usize, usize), T>,
}

impl<T> SymmetricMap<T> {
    pub fn new() -> Self {
        SymmetricMap {
            map: FnvHashMap::default(),
        }
    }

    fn order_indices(i1: usize, i2: usize) -> (usize, usize) {
        if i1 > i2 {
            (i2, i1)
        } else {
            (i1, i2)
        }
    }

    pub fn get(&self, i1: usize, i2: usize) -> &T {
        &self.map[&Self::order_indices(i1, i2)]
    }

    pub fn insert(&mut self, i1: usize, i2: usize, value: T) {
        self.map.insert(Self::order_indices(i1, i2), value);
    }
}
