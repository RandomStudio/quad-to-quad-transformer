# Quad-to-Quad Perspective Transformer
This library does translation of coordinates within one "source quad" to another (the "destination quad").

It is an application of Projective Transformations, particularly useful for 2D "perspective" transformation. For example, if you are tracking someone in a calibrated user-defined quad and you want to normalise this into a quad which is precisely 1x1 units in size.

I developed this library for use in a 2D LIDAR tracking system, https://crates.io/crates/tether-lidar2d-consolidation. (It used to be a module within that application; now it is a separate library.)

## Notes
Based on the Javascript library https://github.com/jlouthan/perspective-transform

I am also indebted to the excellent explanation of "Projective Transformations" at https://blog.mbedded.ninja/mathematics/geometry/projective-transformations/

Other resources:
- https://www.physicsforums.com/threads/transform-that-maps-points-from-any-quad-to-an-reactangle.833996/
- https://docs.rs/projective/0.3.0/projective/trait.Projective.html provides the necessary API - I think what is needed is the 3x3 (or is it 4x4?) matrix to apply to any given point. Could be worked out by replicating https://github.com/jlouthan/perspective-transform/blob/master/dist/perspective-transform.js ?
- https://math.stackexchange.com/questions/296794/finding-the-transform-matrix-from-4-projected-points-with-javascript/339033#339033
- https://stackoverflow.com/questions/14244032/redraw-image-from-3d-perspective-to-2d/14244616#14244616
- https://blog.mbedded.ninja/mathematics/geometry/projective-transformations/
- https://en.wikipedia.org/wiki/Homography#Mathematical_definition
- https://docs.rs/cgmath/0.18.0/cgmath/struct.Perspective.html
- https://franklinta.com/2014/09/08/computing-css-matrix3d-transforms/
- https://yasenh.github.io/post/homogeneous-coordinates/
