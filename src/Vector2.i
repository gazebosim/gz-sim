%module vector2
%{
#include <ignition/math/Vector2.hh>
%} 

// %rename(__plusequal__) operator+=;

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Vector2
    {
      public: Vector2();
      public: Vector2(const T &_x, const T &_y);
      public: Vector2(const Vector2<T> &_v);
      public: virtual ~Vector2();
      public: double Distance(const Vector2 &_pt) const;
      public: void Normalize();
      public: void Set(T _x, T _y);
      public: T Dot(const Vector2<T> &_v) const;
      public: Vector2 operator+(const Vector2 &_v) const;
      public: Vector2 operator-(const Vector2 &_v) const;
      public: const Vector2 operator/(const Vector2 &_v) const;
      public: const Vector2 operator/(T _v) const;
      public: const Vector2 operator*(const Vector2 &_v) const;
      public: const Vector2 operator*(T _v) const;
      public: bool operator==(const Vector2 &_v) const;
      public: bool IsFinite() const;
      public: inline T X() const;
      public: inline T Y() const;
      public: inline void X(const T &_v);
      public: inline void Y(const T &_v);
    };

    %template(Vector2i) Vector2<int>;
    %template(Vector2d) Vector2<double>;
    %template(Vector2f) Vector2<float>;
  }
}
