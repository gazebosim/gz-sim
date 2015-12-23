/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _IGNITION_MASSMATRIX3_HH_
#define _IGNITION_MASSMATRIX3_HH_

#include <algorithm>
#include <string>
#include <vector>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Quaternion.hh"
#include "ignition/math/Vector2.hh"
#include "ignition/math/Vector3.hh"
#include "ignition/math/Matrix3.hh"

namespace ignition
{
  namespace math
  {
    /// \class MassMatrix3 MassMatrix3.hh ignition/math/MassMatrix3.hh
    /// \brief A class for inertial information about a rigid body
    /// consisting of the scalar mass and a 3x3 symmetric moment
    /// of inertia matrix stored as two Vector3's.
    template<typename T>
    class MassMatrix3
    {
      /// \brief Default Constructor
      public: MassMatrix3() : mass(0)
      {}

      /// \brief Constructor.
      /// \param[in] _mass Mass value in kg if using metric.
      /// \param[in] _Ixxyyzz Diagonal moments of inertia.
      /// \param[in] _Ixyxzyz Off-diagonal moments of inertia
      public: MassMatrix3(const T &_mass,
                          const Vector3<T> &_ixxyyzz,
                          const Vector3<T> &_ixyxzyz )
      : mass(_mass), Ixxyyzz(_ixxyyzz), Ixyxzyz(_ixyxzyz)
      {}

      /// \brief Copy constructor.
      /// \param[in] _massMatrix MassMatrix3 element to copy
      public: MassMatrix3(const MassMatrix3<T> &_m)
      : mass(_m.Mass()), Ixxyyzz(_m.DiagonalMoments()),
        Ixyxzyz(_m.OffDiagonalMoments())
      {}

      /// \brief Destructor.
      public: virtual ~MassMatrix3() {}

      /// \brief Set the mass.
      /// \param[in] _m New mass value.
      /// \return True if the MassMatrix3 is valid.
      public: bool Mass(const T &_m)
      {
        this->mass = _m;
        return this->IsValid();
      }

      /// \brief Get the mass
      /// \return The mass value
      public: T Mass() const
      {
        return this->mass;
      }

      /// \brief Set the moment of inertia matrix.
      /// \param[in] _ixx X second moment of inertia (MOI) about x axis.
      /// \param[in] _iyy Y second moment of inertia about y axis.
      /// \param[in] _izz Z second moment of inertia about z axis.
      /// \param[in] _ixy XY inertia.
      /// \param[in] _ixz XZ inertia.
      /// \param[in] _iyz YZ inertia.
      /// \return True if the MassMatrix3 is valid.
      public: bool InertiaMatrix(const T &_ixx, const T &_iyy, const T &_izz,
                                 const T &_ixy, const T &_ixz, const T &_iyz)
      {
        this->Ixxyyzz.Set(_ixx, _iyy, _izz);
        this->Ixyxzyz.Set(_ixy, _ixz, _iyz);
        return this->IsValid();
      }

      /// \brief Get the diagonal moments of inertia (Ixx, Iyy, Izz).
      /// \return The diagonal moments.
      public: Vector3<T> DiagonalMoments() const
      {
        return this->Ixxyyzz;
      }

      /// \brief Get the off-diagonal moments of inertia (Ixy, Ixz, Iyz).
      /// \return The off-diagonal moments of inertia.
      public: Vector3<T> OffDiagonalMoments() const
      {
        return this->Ixyxzyz;
      }

      /// \brief Set the diagonal moments of inertia (Ixx, Iyy, Izz).
      /// \param[in] _ixxyyzz diagonal moments of inertia
      /// \return True if the MassMatrix3 is valid.
      public: bool DiagonalMoments(const Vector3<T> &_ixxyyzz)
      {
        this->Ixxyyzz = _ixxyyzz;
        return this->IsValid();
      }

      /// \brief Set the off-diagonal moments of inertia (Ixy, Ixz, Iyz).
      /// \param[in] _ixyxzyz off-diagonal moments of inertia
      /// \return True if the MassMatrix3 is valid.
      public: bool OffDiagonalMoments(const Vector3<T> &_ixyxzyz)
      {
        this->Ixyxzyz = _ixyxzyz;
        return this->IsValid();
      }

      /// \brief Get IXX
      /// \return IXX value
      public: T IXX() const
      {
        return this->Ixxyyzz[0];
      }

      /// \brief Get IYY
      /// \return IYY value
      public: T IYY() const
      {
        return this->Ixxyyzz[1];
      }

      /// \brief Get IZZ
      /// \return IZZ value
      public: T IZZ() const
      {
        return this->Ixxyyzz[2];
      }

      /// \brief Get IXY
      /// \return IXY value
      public: T IXY() const
      {
        return this->Ixyxzyz[0];
      }

      /// \brief Get IXZ
      /// \return IXZ value
      public: T IXZ() const
      {
        return this->Ixyxzyz[1];
      }

      /// \brief Get IYZ
      /// \return IYZ value
      public: T IYZ() const
      {
        return this->Ixyxzyz[2];
      }

      /// \brief Set IXX
      /// \param[in] _v IXX value
      /// \return True if the MassMatrix3 is valid.
      public: bool IXX(const T &_v)
      {
        this->Ixxyyzz.X(_v);
        return this->IsValid();
      }

      /// \brief Set IYY
      /// \param[in] _v IYY value
      /// \return True if the MassMatrix3 is valid.
      public: bool IYY(const T &_v)
      {
        this->Ixxyyzz.Y(_v);
        return this->IsValid();
      }

      /// \brief Set IZZ
      /// \param[in] _v IZZ value
      /// \return True if the MassMatrix3 is valid.
      public: bool IZZ(const T &_v)
      {
        this->Ixxyyzz.Z(_v);
        return this->IsValid();
      }

      /// \brief Set IXY
      /// \param[in] _v IXY value
      /// \return True if the MassMatrix3 is valid.
      public: bool IXY(const T &_v)
      {
        this->Ixyxzyz.X(_v);
        return this->IsValid();
      }

      /// \brief Set IXZ
      /// \param[in] _v IXZ value
      /// \return True if the MassMatrix3 is valid.
      public: bool IXZ(const T &_v)
      {
        this->Ixyxzyz.Y(_v);
        return this->IsValid();
      }

      /// \brief Set IYZ
      /// \param[in] _v IYZ value
      /// \return True if the MassMatrix3 is valid.
      public: bool IYZ(const T &_v)
      {
        this->Ixyxzyz.Z(_v);
        return this->IsValid();
      }

      /// \brief returns Moments of Inertia as a Matrix3
      /// \return Moments of Inertia as a Matrix3
      public: Matrix3<T> MOI() const
      {
        return Matrix3<T>(
          this->Ixxyyzz[0], this->Ixyxzyz[0], this->Ixyxzyz[1],
          this->Ixyxzyz[0], this->Ixxyyzz[1], this->Ixyxzyz[2],
          this->Ixyxzyz[1], this->Ixyxzyz[2], this->Ixxyyzz[2]);
      }

      /// \brief Sets Moments of Inertia (MOI) from a Matrix3.
      /// Symmetric component of input matrix is used by averaging
      /// off-axis terms.
      /// \param[in] Moments of Inertia as a Matrix3
      /// \return True if the MassMatrix3 is valid.
      public: bool MOI(const Matrix3<T> &_moi)
      {
        this->Ixxyyzz.Set(_moi(0, 0), _moi(1, 1), _moi(2, 2));
        this->Ixyxzyz.Set(
          0.5*(_moi(0, 1) + _moi(1, 0)),
          0.5*(_moi(0, 2) + _moi(2, 0)),
          0.5*(_moi(1, 2) + _moi(2, 1)));
        return this->IsValid();
      }

      /// \brief Equal operator.
      /// \param[in] _massMatrix MassMatrix3 to copy.
      /// \return Reference to this object.
      public: MassMatrix3 &operator=(const MassMatrix3<T> &_massMatrix)
      {
        this->mass = _massMatrix.Mass();
        this->Ixxyyzz = _massMatrix.DiagonalMoments();
        this->Ixyxzyz = _massMatrix.OffDiagonalMoments();

        return *this;
      }

      /// \brief Equality comparison operator.
      /// \param[in] _m MassMatrix3 to copy.
      /// \return true if each component is equal within a default tolerance,
      /// false otherwise
      public: bool operator==(const MassMatrix3<T> &_m) const
      {
        return equal<T>(this->mass, _m.Mass()) &&
               (this->Ixxyyzz == _m.DiagonalMoments()) &&
               (this->Ixyxzyz == _m.OffDiagonalMoments());
      }

      /// \brief Inequality test operator
      /// \param[in] _m MassMatrix3<T> to test
      /// \return True if not equal (using the default tolerance of 1e-6)
      public: bool operator!=(const MassMatrix3<T> &_m) const
      {
        return !(*this == _m);
      }

      /// \brief Verify that inertia values are positive definite
      /// \return True if mass is positive and moment of inertia matrix
      /// is positive definite.
      public: bool IsPositive() const
      {
        // Check if mass and determinants of all upper left submatrices
        // of moment of inertia matrix are positive
        return (this->mass > 0) &&
               (this->IXX() > 0) &&
               (this->IXX()*this->IYY() - std::pow(this->IXY(), 2) > 0) &&
               (this->MOI().Determinant() > 0);
      }

      /// \brief Verify that inertia values are positive definite
      /// and satisfy the triangle inequality.
      /// \return True if IsPositive and moment of inertia satisfies
      /// the triangle inequality.
      public: bool IsValid() const
      {
        return this->IsPositive() && ValidMoments(this->PrincipalMoments());
      }

      /// \brief Verify that principal moments are positive
      /// and satisfy the triangle inequality.
      /// \param[in] _moments Principal moments of inertia.
      /// \return True if moments of inertia are positive
      /// and satisfy the triangle inequality.
      public: static bool ValidMoments(const Vector3<T> &_moments)
      {
        return _moments[0] > 0 &&
               _moments[1] > 0 &&
               _moments[2] > 0 &&
               _moments[0] + _moments[1] > _moments[2] &&
               _moments[1] + _moments[2] > _moments[0] &&
               _moments[2] + _moments[0] > _moments[1];
      }

      /// \brief Compute principal moments of inertia,
      /// which are the eigenvalues of the moment of inertia matrix.
      /// \param[in] _tol Relative tolerance.
      /// \return Principal moments of inertia. If the matrix is
      /// already diagonal, they are returned in the existing order.
      /// Otherwise, the moments are sorted from smallest to largest.
      public: Vector3<T> PrincipalMoments(const T _tol = 1e-6) const
      {
        // Compute tolerance relative to maximum value of inertia diagonal
        T tol = _tol * this->Ixxyyzz.Max();
        if (this->Ixyxzyz.Equal(Vector3<T>::Zero, tol))
        {
          // Matrix is already diagonalized, return diagonal moments
          return this->Ixxyyzz;
        }

        // Algorithm based on http://arxiv.org/abs/1306.6291v4
        // A Method for Fast Diagonalization of a 2x2 or 3x3 Real Symmetric
        // Matrix, by Maarten Kronenburg
        Vector3<T> Id(this->Ixxyyzz);
        Vector3<T> Ip(this->Ixyxzyz);
        // b = Ixx + Iyy + Izz
        T b = Id.Sum();
        // c = Ixx*Iyy - Ixy^2  +  Ixx*Izz - Ixz^2  +  Iyy*Izz - Iyz^2
        T c = Id[0]*Id[1] - std::pow(Ip[0], 2)
            + Id[0]*Id[2] - std::pow(Ip[1], 2)
            + Id[1]*Id[2] - std::pow(Ip[2], 2);
        // d = Ixx*Iyz^2 + Iyy*Ixz^2 + Izz*Ixy^2 - Ixx*Iyy*Izz - 2*Ixy*Ixz*Iyz
        T d = Id[0]*std::pow(Ip[2], 2)
            + Id[1]*std::pow(Ip[1], 2)
            + Id[2]*std::pow(Ip[0], 2)
            - Id[0]*Id[1]*Id[2]
            - 2*Ip[0]*Ip[1]*Ip[2];
        // p = b^2 - 3c
        T p = std::pow(b, 2) - 3*c;

        // At this point, it is important to check that p is not close
        //  to zero, since its inverse is used to compute delta.
        // In equation 4.7, p is expressed as a sum of squares
        //  that is only zero if the matrix is diagonal
        //  with identical principal moments.
        // This check has no test coverage, since this function returns
        //  immediately if a diagonal matrix is detected.
        if (p < std::pow(tol, 2))
          return b / 3.0 * Vector3<T>::One;

        // q = 2b^3 - 9bc - 27d
        T q = 2*std::pow(b, 3) - 9*b*c - 27*d;

        // delta = acos(q / (2 * p^(1.5)))
        // additionally clamp the argument to [-1,1]
        T delta = acos(clamp<T>(0.5 * q / std::pow(p, 1.5), -1, 1));

        // sort the moments from smallest to largest
        T moment0 = (b + 2*sqrt(p) * cos(delta / 3.0)) / 3.0;
        T moment1 = (b + 2*sqrt(p) * cos((delta + 2*M_PI)/3.0)) / 3.0;
        T moment2 = (b + 2*sqrt(p) * cos((delta - 2*M_PI)/3.0)) / 3.0;
        sort3(moment0, moment1, moment2);
        return Vector3<T>(moment0, moment1, moment2);
      }

      /// \brief Mass the object. Default is 0.0.
      private: T mass;

      /// \brief Principal moments of inertia. Default is (0.0 0.0 0.0)
      /// These Moments of Inertia are specified in the local frame.
      /// Where Ixxyyzz.x is Ixx, Ixxyyzz.y is Iyy and Ixxyyzz.z is Izz.
      private: Vector3<T> Ixxyyzz;

      /// \brief Product moments of inertia. Default is (0.0 0.0 0.0)
      /// These MOI off-diagonals are specified in the local frame.
      /// Where Ixyxzyz.x is Ixy, Ixyxzyz.y is Ixz and Ixyxzyz.z is Iyz.
      private: Vector3<T> Ixyxzyz;
    };

    typedef MassMatrix3<double> MassMatrix3d;
    typedef MassMatrix3<float> MassMatrix3f;
  }
}
#endif
