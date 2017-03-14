# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.


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
    + ***Replacement:*** ignition::math::LOw_D

    + ***Deprecation:*** IGN_DBL_INF
    + ***Replacement:*** ignition::math::INF_D

    + ***Deprecation:*** IGN_FLT_MAX
    + ***Replacement:*** ignition::math::MAX_F

    + ***Deprecation:*** IGN_FLT_MIN
    + ***Replacement:*** ignition::math::MIN_F

    + ***Deprecation:*** IGN_FLT_LOW
    + ***Replacement:*** ignition::math::LOw_F

    + ***Deprecation:*** IGN_FLT_INF
    + ***Replacement:*** ignition::math::INF_F

    + ***Deprecation:*** IGN_UI16_MAX
    + ***Replacement:*** ignition::math::MAX_UI16

    + ***Deprecation:*** IGN_UI16_MIN
    + ***Replacement:*** ignition::math::MIN_UI16

    + ***Deprecation:*** IGN_UI16_LOW
    + ***Replacement:*** ignition::math::LOw_UI16

    + ***Deprecation:*** IGN_UI16_INF
    + ***Replacement:*** ignition::math::INF_UI16

    + ***Deprecation:*** IGN_I16_MAX
    + ***Replacement:*** ignition::math::MAX_I16

    + ***Deprecation:*** IGN_I16_MIN
    + ***Replacement:*** ignition::math::MIN_I16

    + ***Deprecation:*** IGN_I16_LOW
    + ***Replacement:*** ignition::math::LOw_I16

    + ***Deprecation:*** IGN_I16_INF
    + ***Replacement:*** ignition::math::INF_I16

    + ***Deprecation:*** IGN_UI32_MAX
    + ***Replacement:*** ignition::math::MAX_UI32

    + ***Deprecation:*** IGN_UI32_MIN
    + ***Replacement:*** ignition::math::MIN_UI32

    + ***Deprecation:*** IGN_UI32_LOW
    + ***Replacement:*** ignition::math::LOw_UI32

    + ***Deprecation:*** IGN_UI32_INF
    + ***Replacement:*** ignition::math::INF_UI32

    + ***Deprecation:*** IGN_I32_MAX
    + ***Replacement:*** ignition::math::MAX_I32

    + ***Deprecation:*** IGN_I32_MIN
    + ***Replacement:*** ignition::math::MIN_I32

    + ***Deprecation:*** IGN_I32_LOW
    + ***Replacement:*** ignition::math::LOw_I32

    + ***Deprecation:*** IGN_I32_INF
    + ***Replacement:*** ignition::math::INF_I32

    + ***Deprecation:*** IGN_UI64_MAX
    + ***Replacement:*** ignition::math::MAX_UI64

    + ***Deprecation:*** IGN_UI64_MIN
    + ***Replacement:*** ignition::math::MIN_UI64

    + ***Deprecation:*** IGN_UI64_LOW
    + ***Replacement:*** ignition::math::LOw_UI64

    + ***Deprecation:*** IGN_UI64_INF
    + ***Replacement:*** ignition::math::INF_UI64

    + ***Deprecation:*** IGN_I64_MAX
    + ***Replacement:*** ignition::math::MAX_I64

    + ***Deprecation:*** IGN_I64_MIN
    + ***Replacement:*** ignition::math::MIN_I64

    + ***Deprecation:*** IGN_I64_LOW
    + ***Replacement:*** ignition::math::LOw_I64

    + ***Deprecation:*** IGN_I64_INF
    + ***Replacement:*** ignition::math::INF_I64

