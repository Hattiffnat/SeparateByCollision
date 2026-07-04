use bvh::{
    bounding_hierarchy::{BHShape, BHValue},
    bvh::{Bvh, BvhNode},
};

pub(crate) fn bvh_vs_bvh<'a, F, const D: usize, T1, T2>(
    bvh_1: &'a Bvh<F, D>,
    bvh_2: &'a Bvh<F, D>,
    shapes_1: &'a [T1],
    shapes_2: &'a [T2],
) -> Vec<(&'a T1, &'a T2)>
where
    F: BHValue,
    T1: BHShape<F, D>,
    T2: BHShape<F, D>,
{
    let mut result = Vec::new();
    let mut stack = vec![(&bvh_1.nodes[0], &bvh_2.nodes[0])];

    while let Some((node_1, node_2)) = stack.pop() {
        if !node_1
            .get_node_aabb(shapes_1)
            .intersects_aabb(&node_2.get_node_aabb(shapes_2))
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
            ) => result.push((&shapes_1[*sh1], &shapes_2[*sh2])),
            (
                BvhNode::Leaf { .. },
                BvhNode::Node {
                    child_l_index,
                    child_r_index,
                    ..
                },
            ) => {
                stack.push((node_1, &bvh_2.nodes[*child_l_index]));
                stack.push((node_1, &bvh_2.nodes[*child_r_index]));
            }
            (
                BvhNode::Node {
                    child_l_index,
                    child_r_index,
                    ..
                },
                BvhNode::Leaf { .. },
            ) => {
                stack.push((&bvh_1.nodes[*child_l_index], node_2));
                stack.push((&bvh_1.nodes[*child_r_index], node_2));
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
                stack.push((&bvh_1.nodes[*ch_l_1], &bvh_2.nodes[*ch_l_2]));
                stack.push((&bvh_1.nodes[*ch_l_1], &bvh_2.nodes[*ch_r_2]));
                stack.push((&bvh_1.nodes[*ch_r_1], &bvh_2.nodes[*ch_l_2]));
                stack.push((&bvh_1.nodes[*ch_r_1], &bvh_2.nodes[*ch_r_2]));
            }
        }
    }

    result
}
