# Quad-to-Quad Perspective Transformer
This library does translation of coordinates within one "source quad" to another (the "destination quad").

It is an application of Projective Transformations, particularly useful for 2D "perspective" transformation. For example, if you are tracking someone in a calibrated user-defined quad and you want to normalise this into a quad which is precisely 1x1 units in size.

I developed this library for use in a 2D LIDAR tracking system, https://crates.io/crates/tether-lidar2d-consolidation. (It used to be a module within that application; now it is a separate library.)

## Notes
Based on the Javascript library https://github.com/jlouthan/perspective-transform

I am also indebted to the excellent explanation of "Projective Transformations" at https://blog.mbedded.ninja/mathematics/geometry/projective-transformations/
