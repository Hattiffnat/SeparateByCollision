use bvh::aabb::Bounded;
use bvh::bounding_hierarchy::BoundingHierarchy;
use bvh::bvh::Bvh;
use itertools::iproduct;
use std::collections::HashMap;

use crate::bvhs_comp;
use crate::shapes_comp::{
    closest_point_on_segment, closest_point_on_tri, closest_points_segment_segment,
    segment_intersects_triangle,
};

use super::Island;

pub(crate) fn island_vs_island<'isl, 'map>(
    isl_1_i: usize,
    isl_2_i: usize,
    islands: &mut [Island],
    edge_bvhs: &'isl mut HashMap<usize, Bvh<f32, 3>>,
    tri_bvhs: &'isl mut HashMap<usize, Bvh<f32, 3>>,
) -> bool {
    let max_distance = islands[isl_1_i].radius + islands[isl_2_i].radius;
    let max_distance_sq = max_distance.powi(2);

    // dbg!(&islands[isl_1_i].edges);
    // dbg!(&islands[isl_1_i].tris);
    // dbg!(&islands[isl_2_i].edges);
    // dbg!(&islands[isl_2_i].tris);

    // SINGLE VERT ISLANDS CASE
    if islands[isl_1_i].verts_inds.len() == 1 && islands[isl_2_i].verts_inds.len() == 1 {
        let p1 = islands[isl_1_i]
            .mesh
            .get_point(&islands[isl_1_i].verts_inds[0]);
        let p2 = islands[isl_2_i]
            .mesh
            .get_point(&islands[isl_2_i].verts_inds[0]);

        return (p1 - p2).magnitude_squared() <= max_distance_sq;
    }

    let (isl_1_i, isl_2_i) =
        if islands[isl_1_i].verts_inds.len() > islands[isl_2_i].verts_inds.len() {
            (isl_2_i, isl_1_i)
        } else {
            (isl_1_i, isl_2_i)
        };

    // SINGLE VERT VS MESH
    if islands[isl_1_i].verts_inds.len() == 1 {
        let p = islands[isl_1_i]
            .mesh
            .get_point(&islands[isl_1_i].verts_inds[0]);

        if !tri_bvhs.contains_key(&isl_2_i) {
            // mut borrowing
            tri_bvhs.insert(isl_2_i, Bvh::build_par(&mut islands[isl_2_i].tris));
        }
        // not mut borrowning
        let tri_bvh_2 = tri_bvhs.get(&isl_2_i).expect("what???");

        for tri in tri_bvh_2.traverse_iterator(&islands[isl_1_i].aabb(), &islands[isl_2_i].tris) {
            let (a, b, c) = tri.get_points_co();

            let closest_p = closest_point_on_tri(p, a, b, c);
            if (closest_p - p).magnitude_squared() <= max_distance_sq {
                return true;
            }
        }

        if !edge_bvhs.contains_key(&isl_2_i) {
            edge_bvhs.insert(isl_2_i, Bvh::build_par(&mut islands[isl_2_i].edges));
        }
        let edge_2_bvh = edge_bvhs.get(&isl_2_i).expect("what???");

        for e in edge_2_bvh.traverse_iterator(&islands[isl_1_i].aabb(), &islands[isl_2_i].edges) {
            let (a, b) = e.get_points_co();
            let closest_p = closest_point_on_segment(p, a, b);

            if (closest_p - p).magnitude_squared() <= max_distance_sq {
                return true;
            }
        }

        return false;
    }

    // TRI VS TRI
    if !tri_bvhs.contains_key(&isl_1_i) {
        tri_bvhs.insert(isl_1_i, Bvh::build_par(&mut islands[isl_1_i].tris));
    };
    if !tri_bvhs.contains_key(&isl_2_i) {
        tri_bvhs.insert(isl_2_i, Bvh::build_par(&mut islands[isl_2_i].tris));
    };

    let tri_1_bvh = tri_bvhs.get(&isl_1_i).expect("what???");
    let tri_2_bvh = tri_bvhs.get(&isl_2_i).expect("what???");

    for (tri_1, tri_2) in bvhs_comp::bvh_vs_bvh(
        tri_1_bvh,
        tri_2_bvh,
        &islands[isl_1_i].tris,
        &islands[isl_2_i].tris,
    ) {
        for (t_1, t_2) in [(tri_1, tri_2), (tri_2, tri_1)] {
            // VERT VS TRI
            for v_i in t_1.verts_inds.iter() {
                let p = t_1.mesh.get_point(v_i);
                let (a, b, c) = t_2.get_points_co();

                let closest = closest_point_on_tri(p, a, b, c);
                if (p - closest).magnitude_squared() <= max_distance_sq {
                    return true;
                }
            }

            // SEGMENT INTERSECTION
            let (t1, t2, t3) = tri_2.get_points_co();
            for (s1, s2) in tri_1.get_segments_co() {
                if segment_intersects_triangle(s1, s2, t1, t2, t3) {
                    return true;
                }
            }
        }
        // TRI SIDE VS TRI SIDE
        for ((e1_p1, e1_p2), (e2_p1, e2_p2)) in
            iproduct!(&tri_1.get_segments_co(), &tri_2.get_segments_co())
        {
            let (p1, p2) = closest_points_segment_segment(e1_p1, e1_p2, e2_p1, e2_p2);

            if (p1 - p2).magnitude_squared() <= max_distance_sq {
                return true;
            }
        }
    }

    // EDGE VS EDGE
    if !edge_bvhs.contains_key(&isl_1_i) {
        edge_bvhs.insert(isl_1_i, Bvh::build_par(&mut islands[isl_1_i].edges));
    };
    if !edge_bvhs.contains_key(&isl_2_i) {
        edge_bvhs.insert(isl_2_i, Bvh::build_par(&mut islands[isl_2_i].edges));
    };

    let edge_1_bvh = edge_bvhs.get(&isl_1_i).expect("what???");
    let edge_2_bvh = edge_bvhs.get(&isl_2_i).expect("what???");

    // EDGE VS TRI
    for (e, t) in bvhs_comp::bvh_vs_bvh(
        edge_1_bvh,
        tri_2_bvh,
        &islands[isl_1_i].edges,
        &islands[isl_2_i].tris,
    )
    .chain(bvhs_comp::bvh_vs_bvh(
        edge_2_bvh,
        tri_1_bvh,
        &islands[isl_2_i].edges,
        &islands[isl_1_i].tris,
    )) {
        let (ep_1, ep_2) = e.get_points_co();
        for (s1, s2) in t.get_segments_co() {
            let (p1, p2) = closest_points_segment_segment(ep_1, ep_2, s1, s2);
            if (p1 - p2).magnitude_squared() <= max_distance_sq {
                return true;
            }

            let (tp_1, tp_2, tp_3) = t.get_points_co();
            if segment_intersects_triangle(ep_1, ep_2, tp_1, tp_2, tp_3) {
                return true;
            }
        }
    }

    // EDGE VS EDGE
    for ((a1, a2), (b1, b2)) in bvhs_comp::bvh_vs_bvh(
        edge_1_bvh,
        edge_2_bvh,
        &islands[isl_1_i].edges,
        &islands[isl_2_i].edges,
    )
    .map(|(e1, e2)| (e1.get_points_co(), e2.get_points_co()))
    {
        let (closest_1, closest_2) = closest_points_segment_segment(a1, a2, b1, b2);
        if (closest_1 - closest_2).magnitude_squared() <= max_distance_sq {
            return true;
        }
    }

    false
}
