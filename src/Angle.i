%module angle
%{
#include <ignition/math/Angle.hh>
%} 

namespace ignition
{
  namespace math
  {
    class Angle
    {
      public: static const Angle Zero;
      public: static const Angle Pi;
      public: static const Angle HalfPi;
      public: static const Angle TwoPi;
      public: Angle();
      public: Angle(double _radian);
      public: Angle(const Angle &_angle);
      public: virtual ~Angle();
      public: void Radian(double _radian);
      public: void Degree(double _degree);
      public: double Radian() const;
      public: double Degree() const;
      public: void Normalize();
      public: inline double operator*() const;
      public: Angle operator-(const Angle &_angle) const;
      public: Angle operator+(const Angle &_angle) const;
      public: Angle operator*(const Angle &_angle) const;
      public: Angle operator/(const Angle &_angle) const;
      public: bool operator==(const Angle &_angle) const;
      public: bool operator<(const Angle &_angle) const;
      public: bool operator<=(const Angle &_angle) const;
      public: bool operator>(const Angle &_angle) const;
      public: bool operator>=(const Angle &_angle) const;
    };
  }
}
