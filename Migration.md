# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Ignition Math 3.X to 4.X

### Added dependencies

1. **ignition-cmake**
    + Ignition-math now has a build dependency on ignition-cmake, which
      allows cmake scripts to be shared across all the ignition packages.

### Modifications

1. **Box.hh**
    + Boxes generated with the default constructor do not intersect any other
    boxes or contain any points (previously they contained the origin).

1. **SemanticVersion.hh**
    + The SemanticVersion(const std::string &) constructor is now explicit.

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

