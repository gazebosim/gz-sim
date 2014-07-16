%module vector2
%{
#include <ignition/math/Vector2.hh>
%} 

namespace ignition
{
  namespace math
  {
    /// \class Vector2 Vector2.hh ignition/math/Vector2.hh
    /// \brief Two dimensional (x, y) vector.
    template<typename T>
    class Vector2
    {
      /// \brief Default Constructor
      public: Vector2();

      /// \brief Constructor
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      public: Vector2(const T &_x, const T &_y);
      
      /// \brief Copy constructor
      /// \param[in] _v the value
      public: Vector2(const Vector2<T> &_v);

      /// \brief Destructor
      public: virtual ~Vector2();

      /// \brief Calc distance to the given point
      /// \param[in] _pt The point to measure to
      /// \return the distance
      public: double Distance(const Vector2 &_pt) const;

      /// \brief  Normalize the vector length
      public: void Normalize();

      /// \brief Set the contents of the vector
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      public: void Set(T _x, T _y);

      /// \brief Get the dot product of this vector and _v
      /// \param[in] _v the vector
      /// \return The dot product
      public: T Dot(const Vector2<T> &_v) const;

      /// \brief Addition operator
      /// \param[in] _v vector to add
      /// \return sum vector
      public: Vector2 operator+(const Vector2 &_v) const;
      
      /// \brief Addition assignment operator
      /// \param[in] _v the vector to add
      // \return this
      public: const Vector2 &operator+=(const Vector2 &_v);

      /// \brief Subtraction operator
      /// \param[in] _v the vector to substract
      /// \return the subtracted vector
      public: Vector2 operator-(const Vector2 &_v) const;

      /// \brief Subtraction assignment operator
      /// \param[in] _v the vector to substract
      /// \return this
      public: const Vector2 &operator-=(const Vector2 &_v);

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _v a vector
      /// \result a result
      public: const Vector2 operator/(const Vector2 &_v) const;

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _v a vector
      /// \return this
      public: const Vector2 &operator/=(const Vector2 &_v);

      /// \brief Division operator
      /// \param[in] _v the value
      /// \return a vector
      public: const Vector2 operator/(T _v) const;

      /// \brief Division operator
      /// \param[in] _v the divisor
      /// \return a vector
      public: const Vector2 &operator/=(T _v);

      /// \brief Multiplication operators
      /// \param[in] _v the vector
      /// \return the result
      public: const Vector2 operator*(const Vector2 &_v) const;

      /// \brief Multiplication assignment operator
      /// \remarks this is an element wise multiplication
      /// \param[in] _v the vector
      /// \return this
      public: const Vector2 &operator*=(const Vector2 &_v);

      /// \brief Multiplication operators
      /// \param[in] _v the scaling factor
      /// \return a scaled vector
      public: const Vector2 operator*(T _v) const;

      /// \brief Multiplication assignment operator
      /// \param[in] _v the scaling factor
      /// \return a scaled vector
      public: const Vector2 &operator*=(T _v);

      /// \brief Equal to operator
      /// \param[in] _v the vector to compare to
      /// \return true if the elements of the 2 vectors are equal within
      /// a tolerence (1e-6)
      public: bool operator==(const Vector2 &_v) const;

      /// \brief Not equal to operator
      /// \return true if elements are of diffent values (tolerence 1e-6)
      public: bool operator!=(const Vector2 &_v) const;

      /// \brief See if a point is finite (e.g., not nan)
      /// \return true if finite, false otherwise
      public: bool IsFinite() const;

      /// \brief Return the x value.
      /// \return Value of the X component.
      /// \throws N/A.
      public: inline T X() const;

      /// \brief Return the y value.
      /// \return Value of the Y component.
      /// \throws N/A.
      public: inline T Y() const;

      /// \brief Set the x value.
      /// \param[in] _v Value for the x component.
      public: inline void X(const T &_v);

      /// \brief Set the y value.
      /// \param[in] _v Value for the y component.
      public: inline void Y(const T &_v);
    };

    %template(Vector2i) Vector2<int>;
    %template(Vector2d) Vector2<double>;
    %template(Vector2f) Vector2<float>;
  }
}
