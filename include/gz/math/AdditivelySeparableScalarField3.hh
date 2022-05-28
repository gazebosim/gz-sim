/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_MATH_SEPARABLE_SCALAR_FIELD3_HH_
#define GZ_MATH_SEPARABLE_SCALAR_FIELD3_HH_

#include <limits>
#include <utility>

#include <gz/math/Region3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /** \class AdditivelySeparableScalarField3\
     * AdditivelySeparableScalarField3.hh\
     * gz/math/AdditivelySeparableScalarField3.hh
     */
    /// \brief The AdditivelySeparableScalarField3 class constructs
    /// a scalar field F in R^3 as a sum of scalar functions i.e.
    /// F(x, y, z) = k (p(x) + q(y) + r(z)).
    ///
    /// \tparam ScalarFunctionT a callable type that taking a single ScalarT
    ///   value as argument and returning another ScalarT value. Additionally:
    ///   - for AdditivelySeparableScalarField3T to have a stream operator
    ///     overload, ScalarFunctionT must implement a
    ///     void Print(std::ostream &, const std::string &) method that streams
    ///     a representation of it using the given string as argument variable
    ///     name;
    ///   - for AdditivelySeparableScalarField3T::Minimum to be callable,
    ///     ScalarFunctionT must implement a
    ///     ScalarT Minimum(const Interval<ScalarT> &, ScalarT &) method that
    ///     computes its minimum in the given interval and returns an argument
    ///     value that yields said minimum.
    /// \tparam ScalarT a numeric type for which std::numeric_limits<> traits
    ///   have been specialized.
    ///
    /// ## Example
    ///
    /// \snippet examples/additively_separable_scalar_field3_example.cc complete
    template<typename ScalarFunctionT, typename ScalarT>
    class AdditivelySeparableScalarField3
    {
      /// \brief Constructor
      /// \param[in] _k scalar constant
      /// \param[in] _p scalar function of x
      /// \param[in] _q scalar function of y
      /// \param[in] _r scalar function of z
      public: AdditivelySeparableScalarField3(
          ScalarT _k, ScalarFunctionT _p,
          ScalarFunctionT _q, ScalarFunctionT _r)
      : k(_k), p(std::move(_p)), q(std::move(_q)), r(std::move(_r))
      {
      }

      /// \brief Evaluate the scalar field at `_point`
      /// \param[in] _point scalar field argument
      /// \return the result of evaluating `F(_point)`
      public: ScalarT Evaluate(const Vector3<ScalarT> &_point) const
      {
        return this->k * (
            this->p(_point.X()) +
            this->q(_point.Y()) +
            this->r(_point.Z()));
      }

      /// \brief Call operator overload
      /// \see SeparableScalarField3::Evaluate()
      /// \param[in] _point scalar field argument
      /// \return the result of evaluating `F(_point)`
      public: ScalarT operator()(const Vector3<ScalarT> &_point) const
      {
        return this->Evaluate(_point);
      }

      /// \brief Compute scalar field minimum in a `_region`
      /// \param[in] _region scalar field argument set to check
      /// \param[out] _pMin scalar field argument that yields
      ///   the minimum, or NaN if `_region` is empty
      /// \return the scalar field minimum in the given `_region`,
      ///   or NaN if `_region` is empty
      public: ScalarT Minimum(const Region3<ScalarT> &_region,
                              Vector3<ScalarT> &_pMin) const
      {
        if (_region.Empty())
        {
          _pMin = Vector3<ScalarT>::NaN;
          return std::numeric_limits<ScalarT>::quiet_NaN();
        }
        return this->k * (
            this->p.Minimum(_region.Ix(), _pMin.X()) +
            this->q.Minimum(_region.Iy(), _pMin.Y()) +
            this->r.Minimum(_region.Iz(), _pMin.Z()));
      }

      /// \brief Compute scalar field minimum in a `_region`
      /// \param[in] _region scalar field argument set to check
      /// \return the scalar field minimum in the given `_region`,
      ///   or NaN if `_region` is empty
      public: ScalarT Minimum(const Region3<ScalarT> &_region) const
      {
        Vector3<ScalarT> pMin;
        return this->Minimum(_region, pMin);
      }

      /// \brief Compute scalar field minimum
      /// \param[out] _pMin scalar field argument that yields
      ///   the minimum, or NaN if `_region` is empty
      /// \return the scalar field minimum
      public: ScalarT Minimum(Vector3<ScalarT> &_pMin) const
      {
        return this->Minimum(Region3<ScalarT>::Unbounded, _pMin);
      }

      /// \brief Compute scalar field minimum
      /// \return the scalar field minimum
      public: ScalarT Minimum() const
      {
        Vector3<ScalarT> pMin;
        return this->Minimum(Region3<ScalarT>::Unbounded, pMin);
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _field SeparableScalarField3 to output
      /// \return the stream
      public: friend std::ostream &operator<<(
          std::ostream &_out,
          const gz::math::AdditivelySeparableScalarField3<
          ScalarFunctionT, ScalarT> &_field)
      {
        using std::abs;  // enable ADL
        constexpr ScalarT epsilon =
            std::numeric_limits<ScalarT>::epsilon();
        if ((abs(_field.k) - ScalarT(1)) < epsilon)
        {
          if (_field.k < ScalarT(0))
          {
            _out << "-";
          }
        }
        else
        {
          _out << _field.k << " ";
        }
        _out << "[(";
        _field.p.Print(_out, "x");
        _out << ") + (";
        _field.q.Print(_out, "y");
        _out << ") + (";
        _field.r.Print(_out, "z");
        return _out << ")]";
      }

      /// \brief Scalar constant
      private: ScalarT k;

      /// \brief Scalar function of x
      private: ScalarFunctionT p;

      /// \brief Scalar function of y
      private: ScalarFunctionT q;

      /// \brief Scalar function of z
      private: ScalarFunctionT r;
    };

    template<typename ScalarFunctionT>
    using AdditivelySeparableScalarField3f =
        AdditivelySeparableScalarField3<ScalarFunctionT, float>;

    template<typename ScalarFunctionT>
    using AdditivelySeparableScalarField3d =
        AdditivelySeparableScalarField3<ScalarFunctionT, double>;
    }
  }
}
#endif
