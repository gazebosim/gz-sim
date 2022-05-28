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
#ifndef GZ_MATH_PIECEWISE_SCALAR_FIELD3_HH_
#define GZ_MATH_PIECEWISE_SCALAR_FIELD3_HH_

#include <algorithm>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

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
    /** \class PiecewiseScalarField3 PiecewiseScalarField3.hh\
     * gz/math/PiecewiseScalarField3.hh
     */
    /// \brief The PiecewiseScalarField3 class constructs a scalar field F
    /// in R^3 as a union of scalar fields Pn, defined over regions Rn i.e.
    /// piecewise.
    ///
    /// \tparam ScalarField3T a callable type taking a single Vector3<ScalarT>
    ///   value as argument and returning a ScalarT value. Additionally:
    ///   - for PiecewiseScalarField3 to have a stream operator overload,
    ///     ScalarField3T must support stream operator overload;
    ///   - for PiecewiseScalarField3::Minimum to be callable, ScalarField3T
    ///     must implement a
    ///     ScalarT Minimum(const Region3<ScalarT> &, Vector3<ScalarT> &)
    ///     method that computes its minimum in the given region and returns
    ///     an argument value that yields said minimum.
    /// \tparam ScalarT a numeric type for which std::numeric_limits<> traits
    ///   have been specialized.
    ///
    /// ## Example
    ///
    /// \snippet examples/piecewise_scalar_field3_example.cc complete
    template<typename ScalarField3T, typename ScalarT>
    class PiecewiseScalarField3
    {
      /// \brief A scalar field P in R^3 and
      /// the region R in which it is defined
      public: struct Piece {
        Region3<ScalarT> region;
        ScalarField3T field;
      };

      /// \brief Constructor
      public: PiecewiseScalarField3() = default;

      /// \brief Constructor
      /// \param[in] _pieces scalar fields Pn and the regions Rn
      ///   in which these are defined. Regions should not overlap.
      public: explicit PiecewiseScalarField3(const std::vector<Piece> &_pieces)
      : pieces(_pieces)
      {
        for (size_t i = 0; i < pieces.size(); ++i)
        {
          if (pieces[i].region.Empty())
          {
            std::cerr << "Region #" << i << " (" << pieces[i].region
                      << ") in piecewise scalar field definition is empty."
                      << std::endl;
          }
          for (size_t j = i + 1; j < pieces.size(); ++j)
          {
            if (pieces[i].region.Intersects(pieces[j].region))
            {
              std::cerr << "Detected overlap between regions in "
                        << "piecewise scalar field definition: "
                        << "region #" << i << " (" << pieces[i].region
                        << ") overlaps with region #" << j << " ("
                        << pieces[j].region << "). Region #" << i
                        << " will take precedence when overlapping."
                        << std::endl;
            }
          }
        }
      }

      /// \brief Define piecewise scalar field as `_field` throughout R^3 space
      /// \param[in] _field a scalar field in R^3
      /// \return `_field` as a piecewise scalar field
      public: static PiecewiseScalarField3 Throughout(ScalarField3T _field)
      {
        return PiecewiseScalarField3<ScalarField3T, ScalarT>({
            {Region3<ScalarT>::Unbounded, std::move(_field)}});
      }

      /// \brief Evaluate the piecewise scalar field at `_p`
      /// \param[in] _p piecewise scalar field argument
      /// \return the result of evaluating `F(_p)`, or NaN
      ///   if the scalar field is not defined at `_p`
      public: ScalarT Evaluate(const Vector3<ScalarT> &_p) const
      {
        auto it = std::find_if(
            this->pieces.begin(), this->pieces.end(),
            [&](const Piece &piece)
            {
              return piece.region.Contains(_p);
            });
        if (it == this->pieces.end())
        {
          return std::numeric_limits<ScalarT>::quiet_NaN();
        }
        return it->field(_p);
      }

      /// \brief Call operator overload
      /// \see PiecewiseScalarField3::Evaluate()
      /// \param[in] _p piecewise scalar field argument
      /// \return the result of evaluating `F(_p)`, or NaN
      ///   if the scalar field is not defined at `_p`
      public: ScalarT operator()(const Vector3<ScalarT> &_p) const
      {
        return this->Evaluate(_p);
      }

      /// \brief Compute the piecewise scalar field minimum
      /// Note that, since this method computes the minimum
      /// for each region independently, it implicitly assumes
      /// continuity in the boundaries between regions, if any.
      /// \param[out] _pMin scalar field argument that yields
      ///   the minimum, or NaN if the scalar field is not
      ///   defined anywhere (i.e. default constructed)
      /// \return the scalar field minimum, or NaN if the
      ///   scalar field is not defined anywhere (i.e. default
      ///   constructed)
      public: ScalarT Minimum(Vector3<ScalarT> &_pMin) const
      {
        if (this->pieces.empty())
        {
          _pMin = Vector3<ScalarT>::NaN;
          return std::numeric_limits<ScalarT>::quiet_NaN();
        }
        ScalarT yMin = std::numeric_limits<ScalarT>::infinity();
        for (const Piece &piece : this->pieces)
        {
          if (!piece.region.Empty())
          {
            Vector3<ScalarT> p;
            const ScalarT y = piece.field.Minimum(piece.region, p);
            if (y < yMin)
            {
              _pMin = p;
              yMin = y;
            }
          }
        }
        return yMin;
      }

      /// \brief Compute the piecewise scalar field minimum
      /// \return the scalar field minimum, or NaN if the
      ///   scalar field is not defined anywhere (i.e. default
      ///   constructed)
      public: ScalarT Minimum() const
      {
        Vector3<ScalarT> pMin;
        return this->Minimum(pMin);
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _field SeparableScalarField3 to output
      /// \return the stream
      public: friend std::ostream &operator<<(
          std::ostream &_out,
          const gz::math::PiecewiseScalarField3<
          ScalarField3T, ScalarT> &_field)
      {
        if (_field.pieces.empty())
        {
          return _out << "undefined";
        }
        for (size_t i = 0; i < _field.pieces.size() - 1; ++i)
        {
          _out << _field.pieces[i].field << " if (x, y, z) in "
               << _field.pieces[i].region << "; ";
        }
        return _out << _field.pieces.back().field
                    << " if (x, y, z) in "
                    << _field.pieces.back().region;
      }

      /// \brief Scalar fields Pn and the regions Rn in which these are defined
      private: std::vector<Piece> pieces;
    };

    template<typename ScalarField3T>
    using PiecewiseScalarField3f = PiecewiseScalarField3<ScalarField3T, float>;
    template<typename ScalarField3T>
    using PiecewiseScalarField3d = PiecewiseScalarField3<ScalarField3T, double>;
    }
  }
}

#endif
