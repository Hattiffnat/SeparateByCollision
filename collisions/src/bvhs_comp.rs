use bvh::{
    bounding_hierarchy::{BHShape, BHValue},
    bvh::{Bvh, BvhNode},
};
use smallvec::{SmallVec, smallvec};

pub struct BvhIntersections<'a, F: BHValue, const D: usize, T1: BHShape<F, D>, T2: BHShape<F, D>> {
    bvh_1: &'a Bvh<F, D>,
    bvh_2: &'a Bvh<F, D>,
    shapes_1: &'a [T1],
    shapes_2: &'a [T2],
    stack: SmallVec<[(&'a BvhNode<F, D>, &'a BvhNode<F, D>); 64]>,
}

impl<'a, F, const D: usize, T1, T2> Iterator for BvhIntersections<'a, F, D, T1, T2>
where
    F: BHValue,
    T1: BHShape<F, D>,
    T2: BHShape<F, D>,
{
    type Item = (&'a T1, &'a T2);

    fn next(&mut self) -> Option<Self::Item> {
        while let Some((node_1, node_2)) = self.stack.pop() {
            if !node_1
                .get_node_aabb(self.shapes_1)
                .intersects_aabb(&node_2.get_node_aabb(self.shapes_2))
            {
                continue;
            }

            match (node_1, node_2) {
                (
                    BvhNode::Leaf {
                        shape_index: sh1, ..
                    },
                    BvhNode::Leaf {
                        shape_index: sh2, ..
                    },
                ) => {
                    return Some((&self.shapes_1[*sh1], &self.shapes_2[*sh2]));
                }
                (
                    BvhNode::Leaf { .. },
                    BvhNode::Node {
                        child_l_index,
                        child_r_index,
                        ..
                    },
                ) => {
                    self.stack.push((node_1, &self.bvh_2.nodes[*child_l_index]));
                    self.stack.push((node_1, &self.bvh_2.nodes[*child_r_index]));
                }
                (
                    BvhNode::Node {
                        child_l_index,
                        child_r_index,
                        ..
                    },
                    BvhNode::Leaf { .. },
                ) => {
                    self.stack.push((&self.bvh_1.nodes[*child_l_index], node_2));
                    self.stack.push((&self.bvh_1.nodes[*child_r_index], node_2));
                }
                (
                    BvhNode::Node {
                        child_l_index: ch_l_1,
                        child_r_index: ch_r_1,
                        ..
                    },
                    BvhNode::Node {
                        child_l_index: ch_l_2,
                        child_r_index: ch_r_2,
                        ..
                    },
                ) => {
                    self.stack
                        .push((&self.bvh_1.nodes[*ch_l_1], &self.bvh_2.nodes[*ch_l_2]));
                    self.stack
                        .push((&self.bvh_1.nodes[*ch_l_1], &self.bvh_2.nodes[*ch_r_2]));
                    self.stack
                        .push((&self.bvh_1.nodes[*ch_r_1], &self.bvh_2.nodes[*ch_l_2]));
                    self.stack
                        .push((&self.bvh_1.nodes[*ch_r_1], &self.bvh_2.nodes[*ch_r_2]));
                }
            }
        }
        None
    }
}

pub(crate) fn bvh_vs_bvh<'a, F, const D: usize, T1, T2>(
    bvh_1: &'a Bvh<F, D>,
    bvh_2: &'a Bvh<F, D>,
    shapes_1: &'a [T1],
    shapes_2: &'a [T2],
) -> BvhIntersections<'a, F, D, T1, T2>
where
    F: BHValue,
    T1: BHShape<F, D>,
    T2: BHShape<F, D>,
{
    let stack = if bvh_1.nodes.len() < 1 || bvh_2.nodes.len() < 1 {
        smallvec![]
    } else {
        smallvec![(&bvh_1.nodes[0], &bvh_2.nodes[0])]
    };

    BvhIntersections {
        bvh_1,
        bvh_2,
        shapes_1,
        shapes_2,
        stack,
    }
}
