use std::collections::HashMap;

// The usual union-find algorithm (disjoint-set data structure)
pub(crate) struct UnionFindManager {
    pub(crate) parents: Vec<usize>,
}

impl UnionFindManager {
    pub(crate) fn new(len: usize) -> Self {
        Self {
            parents: (0..len).collect(),
        }
    }

    pub(crate) fn union(&mut self, a: usize, b: usize) {
        let r_a = self.find(a);
        let r_b = self.find(b);
        if r_a != r_b {
            self.parents[r_b] = r_a;
        }
    }

    pub(crate) fn find(&mut self, a: usize) -> usize {
        let mut x = a;
        while x != self.parents[x] {
            x = self.parents[x];
        }

        self.parents[a] = x;
        x
    }

    pub(crate) fn groups(&mut self) -> HashMap<usize, Vec<usize>> {
        let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();

        for v in 0..self.parents.len() {
            let r = self.find(v);
            groups.entry(r).or_default().push(v);
        }

        groups
    }
}
