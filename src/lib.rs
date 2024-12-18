use anyhow::anyhow;
use log::{debug, warn};
use na::{Matrix3, Point2};

type Point2D = (f32, f32);

extern crate nalgebra as na;

/**
A standardised "1x1 units" box to transform all coordinates into
*/
const DST_SIZE: f32 = 1.;
pub const DEFAULT_DST_QUAD: RectCorners = [
    (0., 0.),
    (DST_SIZE, 0.),
    (DST_SIZE, DST_SIZE),
    (0., DST_SIZE),
];

/**
clockwise: 'left top', 'right top', 'right bottom', 'left bottom',
 */
pub type RectCorners = [Point2D; 4];
type Matrix8x8 = na::SMatrix<f32, 8, 8>;

pub struct QuadTransformer {
    transform_matrix: Option<Matrix3<f32>>,
    ignore_outside_margin: Option<f32>,
    dst_quad: Option<RectCorners>,
}

impl QuadTransformer {
    pub fn new(
        src_quad: Option<RectCorners>,
        dst_quad: Option<RectCorners>,
        ignore_outside_margin: Option<f32>,
    ) -> QuadTransformer {
        if ignore_outside_margin.is_none() {
            warn!("No outside margin value set; points will not be restricted to src_quad");
        }
        if let Some(margin) = ignore_outside_margin {
            warn!("An outside margin value was set; points further than {margin} distance outside of destination quad will be ignored");
        }
        let useable_dst_quad: RectCorners = match dst_quad {
            Some(q) => q,
            None => DEFAULT_DST_QUAD,
        };
        QuadTransformer {
            transform_matrix: src_quad
                .map(|quad| build_transform(&quad.clone(), &useable_dst_quad)),
            dst_quad,
            ignore_outside_margin,
        }
    }

    pub fn set_new_quad(&mut self, src_quad: &RectCorners, dst_quad: Option<RectCorners>) {
        let useable_dst_quad: RectCorners = match dst_quad {
            Some(q) => q,
            None => DEFAULT_DST_QUAD,
        };

        self.dst_quad = dst_quad;

        self.transform_matrix = Some(build_transform(src_quad, &useable_dst_quad));
    }

    /** Take a single input point (within the source quad) and return the
    transformed result (within the destination quad). */
    pub fn transform(&self, point: &Point2D) -> anyhow::Result<Point2D> {
        match self.transform_matrix {
            Some(matrix) => {
                let (x, y) = point;
                let nalgebra_point = Point2::new(*x, *y);

                let transformed = matrix.transform_point(&nalgebra_point);
                Ok((transformed.x, transformed.y))
            }
            None => Err(anyhow!("No transform matrix")),
        }
    }

    /** Using the `ignore_outside_margin` value (if set), return only the points that are
    deemed to be "inside the destination quad". */
    pub fn filter_points_inside(&self, points: &[Point2D]) -> anyhow::Result<Vec<Point2D>> {
        let points: Vec<Point2D> = points
            .iter()
            .filter(|point| match self.ignore_outside_margin {
                Some(margin) => {
                    let (x, y) = point;
                    debug!("...Is {x}, {y} outside of {margin}?");
                    if let Some(dst_quad) = self.dst_quad {
                        let [a, b, c, d] = dst_quad;
                        let d_width = distance(a.0, a.1, b.0, b.1);
                        let d_height = distance(a.0, a.1, d.0, d.1);
                        !(*x > (d_width + margin)
                            || *x < (0. - margin)
                            || *y > (d_height + margin)
                            || *y < (0. - margin))
                    } else {
                        // No destination quad set; use "default" [0;1]
                        !(*x > (DST_SIZE + margin)
                            || *x < (0. - margin)
                            || *y > (DST_SIZE + margin)
                            || *y < (0. - margin))
                    }
                }
                None => true,
            })
            .map(|p| (p.0, p.1))
            .collect();
        Ok(points)
    }

    pub fn is_ready(&self) -> bool {
        self.transform_matrix.is_some()
    }
}

fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    ((x2 - x1).powf(2.0) + (y2 - y1).powf(2.0)).sqrt()
}

fn build_transform(src_quad: &RectCorners, dst_quad: &RectCorners) -> Matrix3<f32> {
    // Mappings by row - each should have 8 terms

    let r1: [f32; 8] = [
        src_quad[0].0,
        src_quad[0].1,
        1.,
        0.,
        0.,
        0.,
        -src_quad[0].0 * dst_quad[0].0,
        -src_quad[0].1 * dst_quad[0].0,
    ];
    let r2: [f32; 8] = [
        0.,
        0.,
        0.,
        src_quad[0].0,
        src_quad[0].1,
        1.,
        -src_quad[0].0 * dst_quad[0].1,
        -src_quad[0].1 * dst_quad[0].1,
    ];
    let r3: [f32; 8] = [
        src_quad[1].0,
        src_quad[1].1,
        1.,
        0.,
        0.,
        0.,
        -src_quad[1].0 * dst_quad[1].0,
        -src_quad[1].1 * dst_quad[1].0,
    ];
    let r4: [f32; 8] = [
        0.,
        0.,
        0.,
        src_quad[1].0,
        src_quad[1].1,
        1.,
        -src_quad[1].0 * dst_quad[1].1,
        -src_quad[1].1 * dst_quad[1].1,
    ];
    let r5: [f32; 8] = [
        src_quad[2].0,
        src_quad[2].1,
        1.,
        0.,
        0.,
        0.,
        -src_quad[2].0 * dst_quad[2].0,
        -src_quad[2].1 * dst_quad[2].0,
    ];
    let r6: [f32; 8] = [
        0.,
        0.,
        0.,
        src_quad[2].0,
        src_quad[2].1,
        1.,
        -src_quad[2].0 * dst_quad[2].1,
        -src_quad[2].1 * dst_quad[2].1,
    ];
    let r7: [f32; 8] = [
        src_quad[3].0,
        src_quad[3].1,
        1.,
        0.,
        0.,
        0.,
        -src_quad[3].0 * dst_quad[3].0,
        -src_quad[3].1 * dst_quad[3].0,
    ];
    let r8: [f32; 8] = [
        0.,
        0.,
        0.,
        src_quad[3].0,
        src_quad[3].1,
        1.,
        -src_quad[3].0 * dst_quad[3].1,
        -src_quad[3].1 * dst_quad[3].1,
    ];
    let combined = vec![r1, r2, r3, r4, r5, r6, r7, r8].into_iter().flatten();

    let matrix_a = Matrix8x8::from_iterator(combined);

    let dst_quad_elements = vec![
        dst_quad[0].0,
        dst_quad[0].1,
        dst_quad[1].0,
        dst_quad[1].1,
        dst_quad[2].0,
        dst_quad[2].1,
        dst_quad[3].0,
        dst_quad[3].1,
    ]
    .into_iter();

    // let matrix_b: na::SMatrix<f32, 1, 8> = na::SMatrix::from_iterator(dst_quad_elements);
    let matrix_b: na::SMatrix<f32, 1, 8> = na::SMatrix::from_iterator(dst_quad_elements);

    // Solve for Ah = B
    let coefficients = matrix_b * matrix_a.try_inverse().unwrap();
    //
    // Create a new 3x3 transform matrix using the elements from above
    Matrix3::new(
        coefficients[0],
        coefficients[1],
        coefficients[2],
        coefficients[3],
        coefficients[4],
        coefficients[5],
        coefficients[6],
        coefficients[7],
        1.,
    )
}

#[cfg(test)]
mod tests {

    use crate::*;

    use super::RectCorners;

    #[test]
    fn test_get_transform_matrix() {
        // numbers as per https://github.com/jlouthan/perspective-transform#basic-usage

        let src_quad = [158., 64., 494., 69., 495., 404., 158., 404.];
        let src_quad: RectCorners = [
            (src_quad[0], src_quad[1]),
            (src_quad[2], src_quad[3]),
            (src_quad[4], src_quad[5]),
            (src_quad[6], src_quad[7]),
        ];

        let dst_quad = [100., 500., 152., 564., 148., 604., 100., 560.];
        let dst_quad: RectCorners = [
            (dst_quad[0], dst_quad[1]),
            (dst_quad[2], dst_quad[3]),
            (dst_quad[4], dst_quad[5]),
            (dst_quad[6], dst_quad[7]),
        ];

        let transform_matrix = build_transform(&src_quad, &dst_quad);

        let src_point = (250., 120.);

        let result = {
            let (x, y) = (src_point.0, src_point.1);
            let nalgebra_point = nalgebra::Point2::new(x, y);

            let transformed = transform_matrix.transform_point(&nalgebra_point);
            (transformed.x, transformed.y)
        };

        assert_eq!(
            (result.0.round(), result.1.round()),
            (
                117.27521125839255_f32.round(),
                530.9202410878403_f32.round(),
            ),
        );
    }

    #[test]
    fn test_get_transform_matrix_simple() {
        // numbers as per https://github.com/jlouthan/perspective-transform#basic-usage

        let src_quad: RectCorners = [(0., 0.), (1., 0.), (1., 1.), (0., 1.)];
        let dst_quad: RectCorners = [(1., 2.), (1., 4.), (3., 4.), (3., 2.)];

        let transform_matrix = build_transform(&src_quad, &dst_quad);

        // The transform matrix is a little different to the example in https://blog.mbedded.ninja/mathematics/geometry/projective-transformations/
        // because their point order is somehow different. The result is the same.
        // assert_eq!(
        //     transform_matrix,
        //     Matrix3::new(2., 0., 1., 0., 2., 2., 0., 0., 1.)
        // );

        let src_point = (0.5, 0.5);

        let result = {
            let (x, y) = (src_point.0, src_point.1);
            let nalgebra_point = nalgebra::Point2::new(x, y);

            let transformed = transform_matrix.transform_point(&nalgebra_point);
            (transformed.x, transformed.y)
        };

        assert_eq!(result, (2., 3.));
    }
}
