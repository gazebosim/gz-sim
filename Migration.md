# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Ignition Math 6.X to 7.X

### Breaking Changes

  1. Removed the Quaternion integer template `Quaternioni`.


### Deprecations

1. **Angle.hh**
    + All mutator functions that lacked a `Set` prefix have been deprecated
    and replaced by version with a `Set` prefix.

1. **Quaternion.hh**
    + ***Deprecation:*** public: void Axis(T, T, T, T)
    + ***Replacement:*** public: void SetFromAxisAngle(T, T, T, T)
    + ***Deprecation:*** public: void Axis(const Vector3<T>&, T)
    + ***Replacement:*** public: void SetFromAxisAngle(const Vector3<T>&, T)
    + ***Deprecation:*** public: void Euler(const Vector3<T> &)
    + ***Replacement:*** public: void SetFromEuler(const Vector3<T> &)
    + ***Deprecation:*** public: void Euler(T, T, T)
    + ***Replacement:*** public: void SetFromEuler(T, T, T)
    + ***Deprecation:*** public: void ToAxis(Vector3<T> &, T &) const
    + ***Replacement:*** public: void void AxisAngle(Vector3<T> &, T &) const
    + ***Deprecation:*** public: void Matrix(const Matrix3<T> &)
    + ***Replacement:*** public: void SetFromMatrix(const Matrix3<T> &)
    + ***Deprecation:*** public: void From2Axes(const Vector3<T> &, const Vector3<T> &)
    + ***Replacement:*** public: void SetFrom2Axes(const Vector3<T> &, const Vector3<T> &)
    + ***Deprecation:*** public: void X(T)
    + ***Replacement:*** public: void SetX(T)
    + ***Deprecation:*** public: void Y(T)
    + ***Replacement:*** public: void SetY(T)
    + ***Deprecation:*** public: void Z(T)
    + ***Replacement:*** public: void SetZ(T)
    + ***Deprecation:*** public: void W(T)
    + ***Replacement:*** public: void SetW(T)

## Ignition Math 4.X to 5.X

### Additions

1. **MassMatrix.hh**
    + Epsilon(const T), Epsilon(const Vector3<T>, const T)
      return relative tolerance proportional to machine
      precision and largest possible moment of inertia.
    * IsNearPositive(const T) is similar to IsPositive(const T)
      but it checks for positive semidefinite inertia
      using >= instead of >.

1. **Plane.hh**
    + Added copy constructor.

1. **Pose3.hh**
    + Added `*=` operator.

### Breaking Changes

1. The `Box` class has been changed to a templatized class that is not
   axis-aligned. The previous `Box` functionality is now in the
   `AxisAlignedBox` class.

1. The behavior of `Pose3` multiplication operator `*` has been changed to
   match behavior of Matrix and Quaternion multiplication.

### Modifications

1. **Inertial.hh**
    + SetMassMatrix now accepts a relative tolerance parameter.
1. **MassMatrix.hh**
    + IsPositive, IsValid, ValidMoments now accept a relative tolerance
      parameter based on Epsilon function.
    + IsValid now uses IsNearPositive instead of IsPositive
    + ValidMoments now uses >= in comparisons instead of >

### Deprecations

1. **MassMatrix3.hh**
    + All mutator functions that lacked a `Set` prefix have been deprecated
    and replaced by version with a `Set` prefix.
    + The MOI functions have been renamed to Moi.
1. **Inertial.hh**
    + The MOI functions have been renamed to Moi.

## Ignition Math 3.X to 4.X

### Added dependencies

1. **ignition-cmake**
    + Ignition-math now has a build dependency on ignition-cmake, which
      allows cmake scripts to be shared across all the ignition packages.

### Modifications

1. **Box.hh**
    + Boxes generated with the default constructor do not intersect any other
    boxes or contain any points (previously they contained the origin).

1. **Helpers.hh**
    + parseInt and parseFloat functions now use std::stoi and std::stod,
      so parsing an alphanumeric string that starts with numbers
      no longer returns a NaN, but instead the beginning of the string
      is parsed (e.g. ("23ab67" -> 23) now, but ("ab23ab67" -> NaN) still).

1. **SemanticVersion.hh**
    + The SemanticVersion(const std::string &) constructor is now explicit.

1. **All Headers**
    + All headers now have an inline versioned namespace. Code should be
    unchanged except all forward declarations of math types must be replaced
    with an include of the header for that type.

### Deprecations

1. **Matrix4.hh**
    + ***Deprecation:*** public: void Translate(const Vector3<T> &_t)
    + ***Replacement:*** public: void SetTranslation(const Vector3<T> &_t)

    + ***Deprecation:*** public: void Translate(T _x, T _y, T _z)
    + ***Replacement:*** public: void SetTranslation(T _x, T _y, T _z)

## Ignition Math 2.X to 3.X

### Modifications

1. **RotationSpline.hh**
    + The `UpdatePoint` function now returns a boolean value.

1. **Spline.hh**
    + The `UpdatePoint` function now returns a boolean value.

### Deprecations

1. **Matrix4.hh**
    + ***Deprecation:*** public: Vector3<T> TransformAffine(const Vector3<T>
        &_v) const
    + ***Replacement:*** public: bool TransformAffine(const Vector3<T>
        &_v,Vector3<T> &_result) const

1. **Helpers.hh**
    + ***Deprecation:*** IGN_DBL_MAX
    + ***Replacement:*** ignition::math::MAX_D

    + ***Deprecation:*** IGN_DBL_MIN
    + ***Replacement:*** ignition::math::MIN_D

    + ***Deprecation:*** IGN_DBL_LOW
    + ***Replacement:*** ignition::math::LOW_D

    + ***Deprecation:*** IGN_DBL_INF
    + ***Replacement:*** ignition::math::INF_D

    + ***Deprecation:*** IGN_FLT_MAX
    + ***Replacement:*** ignition::math::MAX_F

    + ***Deprecation:*** IGN_FLT_MIN
    + ***Replacement:*** ignition::math::MIN_F

    + ***Deprecation:*** IGN_FLT_LOW
    + ***Replacement:*** ignition::math::LOW_F

    + ***Deprecation:*** IGN_FLT_INF
    + ***Replacement:*** ignition::math::INF_F

    + ***Deprecation:*** IGN_UI16_MAX
    + ***Replacement:*** ignition::math::MAX_UI16

    + ***Deprecation:*** IGN_UI16_MIN
    + ***Replacement:*** ignition::math::MIN_UI16

    + ***Deprecation:*** IGN_UI16_LOW
    + ***Replacement:*** ignition::math::LOW_UI16

    + ***Deprecation:*** IGN_UI16_INF
    + ***Replacement:*** ignition::math::INF_UI16

    + ***Deprecation:*** IGN_I16_MAX
    + ***Replacement:*** ignition::math::MAX_I16

    + ***Deprecation:*** IGN_I16_MIN
    + ***Replacement:*** ignition::math::MIN_I16

    + ***Deprecation:*** IGN_I16_LOW
    + ***Replacement:*** ignition::math::LOW_I16

    + ***Deprecation:*** IGN_I16_INF
    + ***Replacement:*** ignition::math::INF_I16

    + ***Deprecation:*** IGN_UI32_MAX
    + ***Replacement:*** ignition::math::MAX_UI32

    + ***Deprecation:*** IGN_UI32_MIN
    + ***Replacement:*** ignition::math::MIN_UI32

    + ***Deprecation:*** IGN_UI32_LOW
    + ***Replacement:*** ignition::math::LOW_UI32

    + ***Deprecation:*** IGN_UI32_INF
    + ***Replacement:*** ignition::math::INF_UI32

    + ***Deprecation:*** IGN_I32_MAX
    + ***Replacement:*** ignition::math::MAX_I32

    + ***Deprecation:*** IGN_I32_MIN
    + ***Replacement:*** ignition::math::MIN_I32

    + ***Deprecation:*** IGN_I32_LOW
    + ***Replacement:*** ignition::math::LOW_I32

    + ***Deprecation:*** IGN_I32_INF
    + ***Replacement:*** ignition::math::INF_I32

    + ***Deprecation:*** IGN_UI64_MAX
    + ***Replacement:*** ignition::math::MAX_UI64

    + ***Deprecation:*** IGN_UI64_MIN
    + ***Replacement:*** ignition::math::MIN_UI64

    + ***Deprecation:*** IGN_UI64_LOW
    + ***Replacement:*** ignition::math::LOW_UI64

    + ***Deprecation:*** IGN_UI64_INF
    + ***Replacement:*** ignition::math::INF_UI64

    + ***Deprecation:*** IGN_I64_MAX
    + ***Replacement:*** ignition::math::MAX_I64

    + ***Deprecation:*** IGN_I64_MIN
    + ***Replacement:*** ignition::math::MIN_I64

    + ***Deprecation:*** IGN_I64_LOW
    + ***Replacement:*** ignition::math::LOW_I64

    + ***Deprecation:*** IGN_I64_INF
    + ***Replacement:*** ignition::math::INF_I64

