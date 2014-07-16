%module vector4
%{
#include <ignition/math/Vector4.hh>
%} 

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Vector4
    {
      public: Vector4();
      public: Vector4(const T &_x, const T &_y, const T &_z, const T &_w);
      public: Vector4(const Vector4<T> &_v);
      public: virtual ~Vector4();
      public: T Distance(const Vector4<T> &_pt) const;
      public: T Length() const;
      public: T SquaredLength() const;
      public: void Normalize();
      public: void Set(T _x = 0, T _y = 0, T _z = 0, T _w = 0);
      public: Vector4<T> &operator=(const Vector4<T> &_v);
      public: Vector4<T> operator+(const Vector4<T> &_v) const;
      public: const Vector4<T> &operator+=(const Vector4<T> &_v);
      public: Vector4<T> operator-(const Vector4<T> &_v) const;
      public: const Vector4<T> &operator-=(const Vector4<T> &_v);
      public: const Vector4<T> operator/(const Vector4<T> &_v) const;
      public: const Vector4<T> &operator/=(const Vector4<T> &_v);
      public: const Vector4<T> operator/(T _v) const;
      public: const Vector4<T> &operator/=(T _v);
      public: const Vector4<T> operator*(const Vector4<T> &_pt) const;
      public: const Vector4<T> operator*(const Matrix4<T> &_m) const;
      public: const Vector4<T> &operator*=(const Vector4<T> &_pt);
      public: const Vector4<T> operator*(T _v) const;
      public: const Vector4<T> &operator*=(T _v);
      public: bool operator==(const Vector4<T> &_v) const;
      public: bool operator!=(const Vector4<T> &_pt) const;
      public: bool IsFinite() const;
      public: inline T operator[](size_t _index) const;
      public: inline T X() const;
      public: inline T Y() const;
      public: inline T Z() const;
      public: inline T W() const;
      public: inline void X(const T &_v);
      public: inline void Y(const T &_v);
      public: inline void Z(const T &_v);
      public: inline void W(const T &_v);
    };

    %template(Vector4i) Vector4<int>;
    %template(Vector4d) Vector4<double>;
    %template(Vector4f) Vector4<float>;
  }
}
