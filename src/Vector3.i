%module vector3
%{
#include <ignition/math/Vector3.hh>
#include <ignition/math/Box.hh>
typedef ignition::math::Vector3<double> Vector3d;
%} 

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Vector3
    {
      public: Vector3();
      public: Vector3(const T &_x, const T &_y, const T &_z);
      public: Vector3(const Vector3<T> &_v);
      public: ~Vector3();
      public: T Sum() const;
      public: T Distance(const Vector3<T> &_pt) const;
      public: T Distance(T _x, T _y, T _z) const;
      public: T Length() const;
      public: T SquaredLength() const;
      public: Vector3 Normalize();
      public: Vector3 Round();
      public: Vector3 Rounded() const;
      public: inline void Set(T _x = 0, T _y = 0, T _z = 0);
      public: Vector3 Cross(const Vector3<T> &_pt) const;
      public: T Dot(const Vector3<T> &_pt) const;
      public: Vector3 Abs() const;
      public: Vector3 Perpendicular() const;
      public: static Vector3 Normal(const Vector3<T> &_v1,
                  const Vector3<T> &_v2, const Vector3<T> &_v3);
      public: T DistToLine(const Vector3<T> &_pt1, const Vector3 &_pt2);
      public: void Max(const Vector3<T> &_v);
      public: void Min(const Vector3<T> &_v);
      public: T Max() const;
      public: T Min() const;
      public: Vector3 operator+(const Vector3<T> &_v) const;
      public: const Vector3 &operator+=(const Vector3<T> &_v);
      public: inline Vector3 operator-() const;
      public: inline Vector3<T> operator-(const Vector3<T> &_pt) const;
      public: const Vector3<T> &operator-=(const Vector3<T> &_pt);
      public: const Vector3<T> operator/(const Vector3<T> &_pt) const;
      public: const Vector3<T> &operator/=(const Vector3<T> &_pt);
      public: const Vector3<T> operator/(T _v) const;
      public: const Vector3<T> &operator/=(T _v);
      public: Vector3<T> operator*(const Vector3<T> &_p) const;
      public: const Vector3<T> &operator*=(const Vector3<T> &_v);
      public: inline Vector3<T> operator*(T _s) const;
      // public: friend inline Vector3<T> operator*(T _s, const Vector3<T> &_v);
      public: const Vector3<T> &operator*=(T _v);
      public: bool operator==(const Vector3<T> &_pt) const;
      public: bool operator!=(const Vector3<T> &_v) const;
      public: bool IsFinite() const;
      public: inline void Correct();
      public: void Round(int _precision);
      public: bool Equal(const Vector3<T> &_v) const;
      public: inline T X() const;
      public: inline T Y() const;
      public: inline T Z() const;
      public: inline void X(const T &_v);
      public: inline void Y(const T &_v);
      public: inline void Z(const T &_v);
    };
    
    %template(Vector3f) Vector3<float>;
    
    class Box
    {
      public: Box();
      public: ignition::math::Vector3d Size() const;
    };

  }
}
