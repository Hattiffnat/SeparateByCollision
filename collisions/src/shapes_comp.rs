use nalgebra::Point3;

type P3 = Point3<f32>;

pub(crate) fn closest_point_on_segment(p: &P3, a: &P3, b: &P3) -> P3 {
    let ab = b - a;
    let ap = p - a;

    let len_sq = ab.magnitude_squared();

    if len_sq == 0. {
        return *a;
    };

    let t = ap.dot(&ab) / len_sq;
    let t = t.clamp(0., 1.);

    return a + ab * t;
}

pub(crate) fn closest_point_on_tri(p: &P3, a: &P3, b: &P3, c: &P3) -> P3 {
    //  pictures/tPiEB.png

    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= (0.).into() && d2 <= 0. {
        return *a;
    } // #1

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0. && d4 <= d3 {
        return *b;
    } // #2

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0. && d5 <= d6 {
        return *c;
    } // #3

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. {
        let v = d1 / (d1 - d3);
        return a + v * ab; // #4
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. {
        let v = d2 / (d2 - d6);
        return a + v * ac; // #5
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0. {
        let v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + v * (c - b); // #6
    }

    let denom = 1. / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;

    a + v * ab + w * ac // #0
}

pub(crate) fn closest_points_segment_segment(a1: &P3, a2: &P3, b1: &P3, b2: &P3) -> (P3, P3) {
    let d1 = a2 - a1;
    let d2 = b2 - b1;
    let r = a1 - b1;

    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    let (s, t) = if a <= f32::EPSILON && e <= f32::EPSILON {
        (0., 0.)
    } else if a <= f32::EPSILON {
        (0., (f / e).clamp(0., 1.))
    } else {
        let c = d1.dot(&r);
        if e <= f32::EPSILON {
            ((-c / a).clamp(0., 1.), 0.)
        } else {
            let b_val = d1.dot(&d2);
            let denom = a * e - b_val * b_val;
            let s = if denom != 0. {
                ((b_val * f - c * e) / denom).clamp(0., 1.)
            } else {
                0.
            };
            let t = (b_val * s + f) / e;
            if t < 0. {
                ((-c / a).clamp(0., 1.), 0.)
            } else if t > 1. {
                (((b_val - c) / a).clamp(0., 1.), 1.)
            } else {
                (s, t)
            }
        }
    };

    (a1 + d1 * s, b1 + d2 * t)
}

pub(crate) fn segment_intersects_triangle(a: &P3, b: &P3, t0: &P3, t1: &P3, t2: &P3) -> bool {
    let dir = b - a;
    let edge1 = t1 - t0;
    let edge2 = t2 - t0;

    let h = dir.cross(&edge2);
    let det = edge1.dot(&h);

    // The ray is parallel to the plane of the triangle
    if det.abs() < f32::EPSILON {
        return false;
    }

    let inv_det = 1.0 / det;
    let s = a - t0;
    let u = inv_det * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return false;
    }

    let q = s.cross(&edge1);
    let v = inv_det * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return false;
    }

    // t = parameter along the segment [a,b], must be in [0,1]
    let t = inv_det * edge2.dot(&q);
    (0.0..=1.0).contains(&t)
}
