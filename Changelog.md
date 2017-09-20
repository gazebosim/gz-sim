## Ignition Math 2.x

1. Fixed frustum falsely saying it contained AABB in some cases
    * [Pull request 193](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/193)

### Ignition Math 2.8

### Ignition Math 2.8.0

1. Added Color
    * [Pull request 150](https://bitbucket.org/ignitionrobotics/ign-math/pull-request/150)

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
