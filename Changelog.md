## Ignition Math 6.x

## Ignition Math 6.x.x

1. Create FAKE_INSTALL target so example compilation can be tested without doing a real install.
    * Angle: [Pull request 335](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/335)

1. Updating documentation.
    * Angle: [Pull request 325](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/325)

1. GraphAlgorithms: add ToUndirected(DirectedGraph) that copies to an UndirectedGraph.
    * [Pull request 332](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/332)

1. Doxygen fixes for graph classes
    * [Pull request 331](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/331)

## Ignition Math 6.4.0

1. Added a function that rounds up a number to the nearest multiple of
   another number.
    * [Pull request 318](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/318)

### Ignition Math 6.3.0

1.  Added Odometry class that computes odometry for a two wheeled vehicle.
    * [Pull request 313](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/313)

1.  Added RollingMean class.
    * [Pull request 314](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/314)

### Ignition Math 6.2.0

1.  eigen3: Use linear() instead of rotation() to prevent computation of SVD
    * [Pull request 311](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/311)

1. Change definition of Pose3 `*` operator to fix multiplication order
    * [Pull request 301](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/301)
    * [Issue 60](https://bitbucket.org/ignitionrobotics/ign-math/issues/60)

### Ignition Math 6.1.0

1. eigen3: add conversion functions for Eigen::AlignedBox3d <=> ignition::math::AxisAlignedBox
    * [Pull request 302](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/302)

### Ignition Math 6.0.0

1. Helper function that converts from `std::chrono::steady_clock::duration` to
   {seconds, nanoseconds}.
    * [Pull request XXX](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/XXX)

1. Upgrade to c++17.
    * [Pull request 268](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/268)

## Ignition Math 5.x

### Ignition Math 5.x.x

### Ignition Math 5.1.0 (2019-09-11)

1. GraphAlgorithms: add ToUndirected(DirectedGraph) that copies to an UndirectedGraph.
    * [Pull request 332](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/332)

1. Doxygen fixes for graph classes
    * [Pull request 331](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/331)

1.  eigen3: Use linear() instead of rotation() to prevent computation of SVD
    * [Pull request 312](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/312)

1. Change definition of Pose3 `*` operator to fix multiplication order
    * [Pull request 301](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/301)
    * [Issue 60](https://bitbucket.org/ignitionrobotics/ign-math/issues/60)

1. eigen3: add conversion functions for Eigen::AlignedBox3d <=> ignition::math::AxisAlignedBox
    * [Pull request 302](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/302)


### Ignition Math 5.0.0 (2018-12-12)

1. Added a Stopwatch class
    * [Pull request 279](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/279)

1. Added material properties to OrientedBox
    * [Pull request 269](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/269)

1. Added a Cylinder class.
    * [Pull request 250](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/250)

1. The `Box` class has been changed to a templatized class that is not
   axis-aligned. The previous `Box` functionality is now in the
   `AxisAlignedBox` class.
    * [Pull request 257](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/257)

1. Added eigen3 component with functions for converting between Eigen and ign-math types.
    * [Pull request 256](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/256)

1. Added a `MassMatrix3::SetFromCylinder` function that uses a `Material`
to specify a density.
    * [Pull request 248](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/248)

1. Added a Sphere class.
    * [Pull request 255](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/255)

1. Added a `MassMatrix3::SetFromSphere` function that uses a `Material` to
specify a density.
    * [Pull request 247](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/247)

1. Added a `MassMatrix3::SetFromBox` function that uses a `Material` to specify
   a density.
    * [Pull request 246](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/246)

1. Deprecated mutator functions in MassMatrix3 that lacked a `Set` prefix.
    * [Pull request 262](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/262)

1. Updated the MassMatrix3::ValidMoments(), MassMatrix3::IsValid(), MassMatrix3::IsPositive(),
 and Inertial::SetMassMatrix functions to accept a tolerance parameter.
    * [Pull request 264](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/264)

1. Add MassMatrix3::IsNearPositive and use it in MassMatrix3::IsValid, use >= instead of >
   in MassMatrix3::ValidMoments
    * [Pull request 278](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/278)

## Ignition Math 4.x

### Ignition Math 4.x.x

1. Add Graph::EdgeFromVertices function that return an edge, if one exists,
   between two vertices.
    * [Pull request 254](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/254)

1. Added multiply assign operator to Matrix4.
    * [Pull request 252](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/252)


1. Add Plane copy constructor and fix cppcheck on artful
    * [Pull request 230](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/230)

1. Added MovingWindowFilter, a copy from Ignition Common. This version will
   replace the version found in Ignition Common.
    * [Pull request 239](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/239)

1. Added a Material class, which holds information about materials like wood,
   steel, and iron.
    * [Pull request 243](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/243)

### Ignition Math 4.0.0 (2017-12-26)

1. Use std::stoi and std::stod in math::parse* functions to reduce code
    * [Pull request 224](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/224)
    * [Issue 50](https://bitbucket.org/ignitionrobotics/ign-math/issues/50)

1. Fixing const-correctness for operator* of Pose3
    * [Pull request 205](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/205)

1. Deprecate Matrix4::Translate and replace by Matrix4::SetTranslation
    * [Pull request 222](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/222)

1. Use ignition-cmake to simplify build scripts
    * [Pull request 200](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/200)

1. Make constructor SemanticVersion(string) explicit
    * [Pull request 203](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/203)

1. Switch to C++14
    * [Pull request 180](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/180)

1. Removed the box 'extent' field. The default constructor now sets a box's
   corners to extrema in order to indicate an uninitialized box.
    * [Pull request 172](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/172)
    * [Issue 72](https://bitbucket.org/ignitionrobotics/ign-math/issues/72)
    * [Issue 53](https://bitbucket.org/ignitionrobotics/ign-math/issues/53)

1. Added graph utilites:
    1. Added a Vertex class:
        * [Pull request 170](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/170)
    1. Added an Edge class:
        * [Pull request 174](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/174)
    1. Added a Graph class:
        * [Pull request 175](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/175)
    1. Added a GraphAlgorithms class:
        * [Pull request 177](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/177)
    1. Added a function to calculate connected components in undirected
       graphs:
        * [Pull request 190](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/190)
    1. Improved the performance of `graph::InDegree()` and `graph::IncidentsTo()`.
        * [Pull request 188](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/188)
        * [Issue 79](https://bitbucket.org/ignitionrobotics/ign-math/issues/79)

1. Added Inline Versioned Namespace
    * [Pull request 216](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/216/)

## Ignition Math 3.x

### Ignition Math 3.x.x



### Ignition Math 3.3.0 (2017-11-27)

1. Fixed frustum falsely saying it contained AABB in some cases
    * [Pull request 193](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/193)
    * [Issue 78](https://bitbucket.org/ignitionrobotics/ign-math/issues/78)

1. Create consistent bracket operators across all Vector# types
    * [Pull request 181](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/181)

1. Change name to the generic BUILDING_DLL macro to avoid conflicts
    * [Pull request 173](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/173)

1. Fix some compiler warnings
    * [Pull request 196](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/196)

1. Suppress gtest warnings
    * [Pull request 199](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/199)

1. Move private headers to src folder
    * [Pull request 198](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/198)

1. Update configure.bat
    * [Pull request 206](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/206)

### Ignition Math 3.2.0 (2017-05-15)

1. Construct on first use in Rand class
    * [Pull request 165](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/165)

1. Extended Spline API: derivative interpolation, arc length calculation
   and tangent forcing.
    * [Pull request 162](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/162)

### Ignition Math 3.1.0 (2017-04-11)

1. Added signum functions to Helpers.hh.
    * Contribution from Martin Pecka
    * [Pull request 153](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/153)

### Ignition Math 3.0.0 (2017-01-05)

1. Deprecate many IGN_* macros in favor of static const variables in Helpers.hh
    * [Pull request 138](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/138)
    * [Pull request 137](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/137)

1. Removed exceptions. Return values should be evaluated to determine if
   errors have occured.
    * [Pull request 132](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/132)

1. Added `operator=(const Quaternion<T> &_q)` to `Matrix3`.
    * [Pull request 111](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/111)

1. Fix xenial cppcheck
    * [Pull request xxx](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/xxx)

1. Require cmake 2.8.12
    * [Pull request 76](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/76)

1. Migrate to relocatable CMake package.
   Contribution from Silvio Traversaro.
    * [Pull request 67](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/67)

1. Fix logic of installation of CMake configuration files in Windows.
   Contribution from Silvio Traversaro.
    * [Pull request 63](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/63)

## Ignition Math 2.x



## Ignition Math 2.9 (2017-11-22)

1. Fixed frustum falsely saying it contained AABB in some cases
    * [Pull request 193](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/193)

1. Added Color
    * [Pull request 150](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/150)

1. Backport updated configure.bat to ign-math2 and fix cppcheck warnings
    * [Pull request 207](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/207)

### Ignition Math 2.8

### Ignition Math 2.8.0

1. Added OrientedBox
    * [Pull request 146](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/146)

1. Added an assignment operator to the Frustum class.
    * [Pull request 144](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/144)

### Ignition Math 2.7

### Ignition Math 2.7.0

1. Add static const variables as alternative to macros in Helpers.hh
    * [Pull request 137](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/137)

1. Add new methods for floating numbers: lessOrEqual and greaterOrEqual
    * [Pull request 134](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/134)

### Ignition Math 2.6

### Ignition Math 2.6.0

1. Added copy constructor, equality operators and assignment operators to
    SphericalCoordinates class.
    * [Pull request 131](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/131)

1. Fix Euler angle conversion of quaternions near singularities
    * [Pull request 129](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/129)

1. Backport triangle3, helper functions, equality helper to work with 387 fp unit
   (Contribution from Rich Mattes).
    * [Pull request 125](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/125)
    * [Pull request 58](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/58)
    * [Pull request 56](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/56)

1. Added Matrix4<T>::LookAt
    * [Pull request 124](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/124)

1. Set Inertial Rotations
    * [Pull request 121](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/121)

1. Added SemanticVersion class
    * [Pull request 120](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/120)

### Ignition Math 2.5

### Ignition Math 2.5.0

1. Added PID class
    * [Pull request 117](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/117)

1. Added SphericalCoordinate class
    * [Pull request 108](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/108)

### Ignition Math 2.4

#### Ignition Math 2.4.1

1. Combine inertial properties of different objects, returning the equivalent
   inertial properties as if the objects were welded together.
    * [Pull request 115](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/115)

#### Ignition Math 2.4.0

1. New MassMatrix3 class
    * [Pull request 112](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/112)
1. MassMatrix3 helper functions
    * [Pull request 110](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/110)
1. Added Temperature class
    * A contribution from Shintaro Noda
    * [Pull request 113](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/113)

### Ignition Math 2.3.0

1. Added simple volumes formulas
    * [Pull request 84](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/84)
1. Add Length and SquaredLength for Vector2 with test
    * [Pull request 73](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/73)
1. Add Equal function with numerical tolerance argument
    * [Pull request 75](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/75)
1. First part of MassMatrix3 class, mostly accessors and modifiers
    * [Pull request 77](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/77)
1. Add Transpose methods for Matrix3,4 with test
    * [Pull request 74](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/74)
1. Multiplication improvements for Vector/Matrix classes
    * [Pull request 69](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/69)
1. Scalar +,- operators for Vector[234]
    * [Pull request 71](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/71)
1. Add Determinant method for Matrix[34]
    * [Pull request 72](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/72)
1. Fixes for compiling and running tests on Windows 7/Visual Studio 2013
   Contribution from Silvio Traversaro.
    * [Pull request 62](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/62)
