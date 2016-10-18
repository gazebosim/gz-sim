/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_MASSMATRIX3_HH_
#define IGNITION_MATH_MASSMATRIX3_HH_

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
      /// \param[in] _ixxyyzz Diagonal moments of inertia.
      /// \param[in] _ixyxzyz Off-diagonal moments of inertia
      public: MassMatrix3(const T &_mass,
                          const Vector3<T> &_ixxyyzz,
                          const Vector3<T> &_ixyxzyz)
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
      /// \param[in] _tol Relative tolerance given by absolute value
      /// of _tol.
      /// Negative values of _tol are interpreted as a flag that
      /// causes principal moments to always be sorted from smallest
      /// to largest.
      /// \return Principal moments of inertia.
      /// If the matrix is already diagonal and _tol is positive,
      /// they are returned in the existing order.
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
        T moment1 = (b + 2*sqrt(p) * cos((delta + 2*IGN_PI)/3.0)) / 3.0;
        T moment2 = (b + 2*sqrt(p) * cos((delta - 2*IGN_PI)/3.0)) / 3.0;
        sort3(moment0, moment1, moment2);
        return Vector3<T>(moment0, moment1, moment2);
      }

      /// \brief Compute rotational offset of principal axes.
      /// \param[in] _tol Relative tolerance given by absolute value
      /// of _tol.
      /// Negative values of _tol are interpreted as a flag that
      /// causes principal moments to always be sorted from smallest
      /// to largest.
      /// \return Quaternion representing rotational offset of principal axes.
      /// With a rotation matrix constructed from this quaternion R(q)
      /// and a diagonal matrix L with principal moments on the diagonal,
      /// the original moment of inertia matrix MOI can be reconstructed
      /// with MOI = R(q).Transpose() * L * R(q)
      public: Quaternion<T> PrincipalAxesOffset(const T _tol = 1e-6) const
      {
        // Compute tolerance relative to maximum value of inertia diagonal
        T tol = _tol * this->Ixxyyzz.Max();
        Vector3<T> moments = this->PrincipalMoments(tol);
        if (moments.Equal(this->Ixxyyzz, tol) ||
            (math::equal<T>(moments[0], moments[1], std::abs(tol)) &&
             math::equal<T>(moments[0], moments[2], std::abs(tol))))
        {
          // matrix is already aligned with principal axes
          // or all three moments are approximately equal
          // return identity rotation
          return Quaternion<T>::Identity;
        }

        // Algorithm based on http://arxiv.org/abs/1306.6291v4
        // A Method for Fast Diagonalization of a 2x2 or 3x3 Real Symmetric
        // Matrix, by Maarten Kronenburg
        // A real, symmetric matrix can be diagonalized by an orthogonal matrix
        // (due to the finite-dimensional spectral theorem
        // https://en.wikipedia.org/wiki/Spectral_theorem
        // #Hermitian_maps_and_Hermitian_matrices ),
        // and another name for orthogonal matrix is rotation matrix.
        // Section 5 of the paper shows how to compute Euler angles
        // phi1, phi2, and phi3 that map to a rotation matrix.
        // In some cases, there are multiple possible values for a given angle,
        // such as phi1, that are denoted as phi11, phi12, phi11a, phi12a, etc.
        // Similar variable names are used to the paper so that the paper
        // can be used as an additional reference.

        // f1, f2 defined in equations 5.5, 5.6
        Vector2<T> f1(this->Ixyxzyz[0], -this->Ixyxzyz[1]);
        Vector2<T> f2(this->Ixxyyzz[1] - this->Ixxyyzz[2],
                   -2*this->Ixyxzyz[2]);

        // Check if two moments are equal, since different equations are used
        // The moments vector is already sorted, so just check adjacent values.
        Vector2<T> momentsDiff(moments[0] - moments[1],
                               moments[1] - moments[2]);

        // index of unequal moment
        int unequalMoment = -1;
        if (equal<T>(momentsDiff[0], 0, std::abs(tol)))
          unequalMoment = 2;
        else if (equal<T>(momentsDiff[1], 0, std::abs(tol)))
          unequalMoment = 0;

        if (unequalMoment >= 0)
        {
          // moments[1] is the repeated value
          // it is not equal to moments[unequalMoment]
          // momentsDiff3 = lambda - lambda3
          T momentsDiff3 = moments[1] - moments[unequalMoment];
          // eq 5.21:
          // s = cos(phi2)^2 = (A_11 - lambda3) / (lambda - lambda3)
          // s >= 0 since A_11 is in range [lambda, lambda3]
          T s = (this->Ixxyyzz[0] - moments[unequalMoment]) / momentsDiff3;
          // set phi3 to zero for repeated moments (eq 5.23)
          T phi3 = 0;
          // phi2 = +- acos(sqrt(s))
          // start with just the positive value
          // also clamp the acos argument to prevent NaN's
          T phi2 = acos(clamp<T>(ClampedSqrt(s), -1, 1));

          // The paper defines variables phi11 and phi12
          // which are candidate values of angle phi1.
          // phi12 is straightforward to compute as a function of f2 and g2.
          // eq 5.25:
          Vector2<T> g2(momentsDiff3 * s, 0);
          // combining eq 5.12 and 5.14, and subtracting psi2
          // instead of multiplying by its rotation matrix:
          math::Angle phi12(0.5*(Angle2(g2, tol) - Angle2(f2, tol)));
          phi12.Normalize();

          // The paragraph prior to equation 5.16 describes how to choose
          // the candidate value of phi1 based on the length
          // of the f1 and f2 vectors.
          // * When |f1| != 0 and |f2| != 0, then one should choose the
          //   value of phi2 so that phi11 = phi12
          // * When |f1| == 0 and f2 != 0, then phi1 = phi12
          //   phi11 can be ignored, and either sign of phi2 can be used
          // * The case of |f2| == 0 can be ignored at this point in the code
          //   since having a repeated moment when |f2| == 0 implies that
          //   the matrix is diagonal. But this function returns a unit
          //   quaternion for diagonal matrices, so we can assume |f2| != 0
          //   See MassMatrix3.ipynb for a more complete discussion.
          //
          // Since |f2| != 0, we only need to consider |f1|
          // * |f1| == 0: phi1 = phi12
          // * |f1| != 0: choose phi2 so that phi11 == phi12
          // In either case, phi1 = phi12,
          // and the sign of phi2 must be chosen to make phi11 == phi12
          T phi1 = phi12.Radian();

          bool f1small = f1.SquaredLength() < std::pow(tol, 2);
          if (!f1small)
          {
            // a: phi2 > 0
            // eq. 5.24
            Vector2<T> g1a(0, 0.5*momentsDiff3 * sin(2*phi2));
            // combining eq 5.11 and 5.13, and subtracting psi1
            // instead of multiplying by its rotation matrix:
            math::Angle phi11a(Angle2(g1a, tol) - Angle2(f1, tol));
            phi11a.Normalize();

            // b: phi2 < 0
            // eq. 5.24
            Vector2<T> g1b(0, 0.5*momentsDiff3 * sin(-2*phi2));
            // combining eq 5.11 and 5.13, and subtracting psi1
            // instead of multiplying by its rotation matrix:
            math::Angle phi11b(Angle2(g1b, tol) - Angle2(f1, tol));
            phi11b.Normalize();

            // choose sign of phi2
            // based on whether phi11a or phi11b is closer to phi12
            // use sin and cos to account for angle wrapping
            T erra = std::pow(sin(phi1) - sin(phi11a.Radian()), 2)
                   + std::pow(cos(phi1) - cos(phi11a.Radian()), 2);
            T errb = std::pow(sin(phi1) - sin(phi11b.Radian()), 2)
                   + std::pow(cos(phi1) - cos(phi11b.Radian()), 2);
            if (errb < erra)
            {
              phi2 *= -1;
            }
          }

          // I determined these arguments using trial and error
          Quaternion<T> result = Quaternion<T>(-phi1, -phi2, -phi3).Inverse();

          // Previous equations assume repeated moments are at the beginning
          // of the moments vector (moments[0] == moments[1]).
          // We have the vectors sorted by size, so it's possible that the
          // repeated moments are at the end (moments[1] == moments[2]).
          // In this case (unequalMoment == 0), we apply an extra
          // rotation that exchanges moment[0] and moment[2]
          // Rotation matrix = [ 0  0  1]
          //                   [ 0  1  0]
          //                   [-1  0  0]
          // That is equivalent to a 90 degree pitch
          if (unequalMoment == 0)
            result *= Quaternion<T>(0, IGN_PI_2, 0);

          return result;
        }

        // No repeated principal moments
        // eq 5.1:
        T v = (std::pow(this->Ixyxzyz[0], 2) + std::pow(this->Ixyxzyz[1], 2)
              +(this->Ixxyyzz[0] - moments[2])
              *(this->Ixxyyzz[0] + moments[2] - moments[0] - moments[1]))
            / ((moments[1] - moments[2]) * (moments[2] - moments[0]));
        // value of w depends on v
        T w;
        if (v < std::abs(tol))
        {
          // first sentence after eq 5.4:
          // "In the case that v = 0, then w = 1."
          w = 1;
        }
        else
        {
          // eq 5.2:
          w = (this->Ixxyyzz[0] - moments[2] + (moments[2] - moments[1])*v)
              / ((moments[0] - moments[1]) * v);
        }
        // initialize values of angle phi1, phi2, phi3
        T phi1 = 0;
        // eq 5.3: start with positive value
        T phi2 = acos(clamp<T>(ClampedSqrt(v), -1, 1));
        // eq 5.4: start with positive value
        T phi3 = acos(clamp<T>(ClampedSqrt(w), -1, 1));

        // compute g1, g2 for phi2,phi3 >= 0
        // equations 5.7, 5.8
        Vector2<T> g1(
          0.5* (moments[0]-moments[1])*ClampedSqrt(v)*sin(2*phi3),
          0.5*((moments[0]-moments[1])*w + moments[1]-moments[2])*sin(2*phi2));
        Vector2<T> g2(
          (moments[0]-moments[1])*(1 + (v-2)*w) + (moments[1]-moments[2])*v,
          (moments[0]-moments[1])*sin(phi2)*sin(2*phi3));

        // The paragraph prior to equation 5.16 describes how to choose
        // the candidate value of phi1 based on the length
        // of the f1 and f2 vectors.
        // * The case of |f1| == |f2| == 0 implies a repeated moment,
        //   which should not be possible at this point in the code
        // * When |f1| != 0 and |f2| != 0, then one should choose the
        //   value of phi2 so that phi11 = phi12
        // * When |f1| == 0 and f2 != 0, then phi1 = phi12
        //   phi11 can be ignored, and either sign of phi2, phi3 can be used
        // * When |f2| == 0 and f1 != 0, then phi1 = phi11
        //   phi12 can be ignored, and either sign of phi2, phi3 can be used
        bool f1small = f1.SquaredLength() < std::pow(tol, 2);
        bool f2small = f2.SquaredLength() < std::pow(tol, 2);
        if (f1small && f2small)
        {
          // this should never happen
          // f1small && f2small implies a repeated moment
          // return invalid quaternion
          return Quaternion<T>::Zero;
        }
        else if (f1small)
        {
          // use phi12 (equations 5.12, 5.14)
          math::Angle phi12(0.5*(Angle2(g2, tol) - Angle2(f2, tol)));
          phi12.Normalize();
          phi1 = phi12.Radian();
        }
        else if (f2small)
        {
          // use phi11 (equations 5.11, 5.13)
          math::Angle phi11(Angle2(g1, tol) - Angle2(f1, tol));
          phi11.Normalize();
          phi1 = phi11.Radian();
        }
        else
        {
          // check for when phi11 == phi12
          // eqs 5.11, 5.13:
          math::Angle phi11(Angle2(g1, tol) - Angle2(f1, tol));
          phi11.Normalize();
          // eqs 5.12, 5.14:
          math::Angle phi12(0.5*(Angle2(g2, tol) - Angle2(f2, tol)));
          phi12.Normalize();
          T err  = std::pow(sin(phi11.Radian()) - sin(phi12.Radian()), 2)
                 + std::pow(cos(phi11.Radian()) - cos(phi12.Radian()), 2);
          phi1 = phi11.Radian();
          math::Vector2<T> signsPhi23(1, 1);
          // case a: phi2 <= 0
          {
            Vector2<T> g1a = Vector2<T>(1, -1) * g1;
            Vector2<T> g2a = Vector2<T>(1, -1) * g2;
            math::Angle phi11a(Angle2(g1a, tol) - Angle2(f1, tol));
            math::Angle phi12a(0.5*(Angle2(g2a, tol) - Angle2(f2, tol)));
            phi11a.Normalize();
            phi12a.Normalize();
            T erra = std::pow(sin(phi11a.Radian()) - sin(phi12a.Radian()), 2)
                   + std::pow(cos(phi11a.Radian()) - cos(phi12a.Radian()), 2);
            if (erra < err)
            {
              err = erra;
              phi1 = phi11a.Radian();
              signsPhi23.Set(-1, 1);
            }
          }
          // case b: phi3 <= 0
          {
            Vector2<T> g1b = Vector2<T>(-1, 1) * g1;
            Vector2<T> g2b = Vector2<T>(1, -1) * g2;
            math::Angle phi11b(Angle2(g1b, tol) - Angle2(f1, tol));
            math::Angle phi12b(0.5*(Angle2(g2b, tol) - Angle2(f2, tol)));
            phi11b.Normalize();
            phi12b.Normalize();
            T errb = std::pow(sin(phi11b.Radian()) - sin(phi12b.Radian()), 2)
                   + std::pow(cos(phi11b.Radian()) - cos(phi12b.Radian()), 2);
            if (errb < err)
            {
              err = errb;
              phi1 = phi11b.Radian();
              signsPhi23.Set(1, -1);
            }
          }
          // case c: phi2,phi3 <= 0
          {
            Vector2<T> g1c = Vector2<T>(-1, -1) * g1;
            Vector2<T> g2c = g2;
            math::Angle phi11c(Angle2(g1c, tol) - Angle2(f1, tol));
            math::Angle phi12c(0.5*(Angle2(g2c, tol) - Angle2(f2, tol)));
            phi11c.Normalize();
            phi12c.Normalize();
            T errc = std::pow(sin(phi11c.Radian()) - sin(phi12c.Radian()), 2)
                   + std::pow(cos(phi11c.Radian()) - cos(phi12c.Radian()), 2);
            if (errc < err)
            {
              err = errc;
              phi1 = phi11c.Radian();
              signsPhi23.Set(-1, -1);
            }
          }

          // apply sign changes
          phi2 *= signsPhi23[0];
          phi3 *= signsPhi23[1];
        }

        // I determined these arguments using trial and error
        return Quaternion<T>(-phi1, -phi2, -phi3).Inverse();
      }

      /// \brief Get dimensions and rotation offset of uniform box
      /// with equivalent mass and moment of inertia.
      /// To compute this, the Matrix3 is diagonalized.
      /// The eigenvalues on the diagonal and the rotation offset
      /// of the principal axes are returned.
      /// \param[in] _size Dimensions of box aligned with principal axes.
      /// \param[in] _rot Rotational offset of principal axes.
      /// \param[in] _tol Relative tolerance.
      /// \return True if box properties were computed successfully.
      public: bool EquivalentBox(Vector3<T> &_size,
                                 Quaternion<T> &_rot,
                                 const T _tol = 1e-6) const
      {
        if (!this->IsPositive())
        {
          // inertia is not positive, cannot compute equivalent box
          return false;
        }

        Vector3<T> moments = this->PrincipalMoments(_tol);
        if (!ValidMoments(moments))
        {
          // principal moments don't satisfy the triangle identity
          return false;
        }

        // The reason for checking that the principal moments satisfy
        // the triangle inequality
        // I1 + I2 - I3 >= 0
        // is to ensure that the arguments to sqrt in these equations
        // are positive and the box size is real.
        _size.X(sqrt(6*(moments.Y() + moments.Z() - moments.X()) / this->mass));
        _size.Y(sqrt(6*(moments.Z() + moments.X() - moments.Y()) / this->mass));
        _size.Z(sqrt(6*(moments.X() + moments.Y() - moments.Z()) / this->mass));

        _rot = this->PrincipalAxesOffset(_tol);

        if (_rot == Quaternion<T>::Zero)
        {
          // _rot is an invalid quaternion
          return false;
        }

        return true;
      }

      /// \brief Set inertial properties based on mass and equivalent box.
      /// \param[in] _mass Mass to set.
      /// \param[in] _size Size of equivalent box.
      /// \param[in] _rot Rotational offset of equivalent box.
      /// \return True if inertial properties were set successfully.
      public: bool SetFromBox(const T _mass,
                              const Vector3<T> &_size,
                            const Quaternion<T> &_rot = Quaternion<T>::Identity)
      {
        // Check that _mass and _size are strictly positive
        // and that quatenion is valid
        if (_mass <= 0 || _size.Min() <= 0 || _rot == Quaternion<T>::Zero)
        {
          return false;
        }
        this->Mass(_mass);
        return this->SetFromBox(_size, _rot);
      }

      /// \brief Set inertial properties based on equivalent box
      /// using the current mass value.
      /// \param[in] _size Size of equivalent box.
      /// \param[in] _rot Rotational offset of equivalent box.
      /// \return True if inertial properties were set successfully.
      public: bool SetFromBox(const Vector3<T> &_size,
                            const Quaternion<T> &_rot = Quaternion<T>::Identity)
      {
        // Check that _mass and _size are strictly positive
        // and that quatenion is valid
        if (this->Mass() <= 0 || _size.Min() <= 0 ||
            _rot == Quaternion<T>::Zero)
        {
          return false;
        }

        // Diagonal matrix L with principal moments
        Matrix3<T> L;
        T x2 = std::pow(_size.X(), 2);
        T y2 = std::pow(_size.Y(), 2);
        T z2 = std::pow(_size.Z(), 2);
        L(0, 0) = this->mass / 12.0 * (y2 + z2);
        L(1, 1) = this->mass / 12.0 * (z2 + x2);
        L(2, 2) = this->mass / 12.0 * (x2 + y2);
        Matrix3<T> R(_rot);
        return this->MOI(R * L * R.Transposed());
      }

      /// \brief Set inertial properties based on mass and equivalent cylinder
      /// aligned with Z axis.
      /// \param[in] _mass Mass to set.
      /// \param[in] _length Length of cylinder along Z axis.
      /// \param[in] _radius Radius of cylinder.
      /// \param[in] _rot Rotational offset of equivalent cylinder.
      /// \return True if inertial properties were set successfully.
      public: bool SetFromCylinderZ(const T _mass,
                                    const T _length,
                                    const T _radius,
                            const Quaternion<T> &_rot = Quaternion<T>::Identity)
      {
        // Check that _mass, _radius and _length are strictly positive
        // and that quatenion is valid
        if (_mass <= 0 || _length <= 0 || _radius <= 0 ||
            _rot == Quaternion<T>::Zero)
        {
          return false;
        }
        this->Mass(_mass);
        return this->SetFromCylinderZ(_length, _radius, _rot);
      }

      /// \brief Set inertial properties based on equivalent cylinder
      /// aligned with Z axis using the current mass value.
      /// \param[in] _length Length of cylinder along Z axis.
      /// \param[in] _radius Radius of cylinder.
      /// \param[in] _rot Rotational offset of equivalent cylinder.
      /// \return True if inertial properties were set successfully.
      public: bool SetFromCylinderZ(const T _length,
                                    const T _radius,
                                    const Quaternion<T> &_rot)
      {
        // Check that _mass and _size are strictly positive
        // and that quatenion is valid
        if (this->Mass() <= 0 || _length <= 0 || _radius <= 0 ||
            _rot == Quaternion<T>::Zero)
        {
          return false;
        }

        // Diagonal matrix L with principal moments
        T radius2 = std::pow(_radius, 2);
        Matrix3<T> L;
        L(0, 0) = this->mass / 12.0 * (3*radius2 + std::pow(_length, 2));
        L(1, 1) = L(0, 0);
        L(2, 2) = this->mass / 2.0 * radius2;
        Matrix3<T> R(_rot);
        return this->MOI(R * L * R.Transposed());
      }

      /// \brief Set inertial properties based on mass and equivalent sphere.
      /// \param[in] _mass Mass to set.
      /// \param[in] _radius Radius of equivalent, uniform sphere.
      /// \return True if inertial properties were set successfully.
      public: bool SetFromSphere(const T _mass, const T _radius)
      {
        // Check that _mass and _radius are strictly positive
        if (_mass <= 0 || _radius <= 0)
        {
          return false;
        }
        this->Mass(_mass);
        return this->SetFromSphere(_radius);
      }

      /// \brief Set inertial properties based on equivalent sphere
      /// using the current mass value.
      /// \param[in] _radius Radius of equivalent, uniform sphere.
      /// \return True if inertial properties were set successfully.
      public: bool SetFromSphere(const T _radius)
      {
        // Check that _mass and _radius are strictly positive
        if (this->Mass() <= 0 || _radius <= 0)
        {
          return false;
        }

        // Diagonal matrix L with principal moments
        T radius2 = std::pow(_radius, 2);
        Matrix3<T> L;
        L(0, 0) = 0.4 * this->mass * radius2;
        L(1, 1) = 0.4 * this->mass * radius2;
        L(2, 2) = 0.4 * this->mass * radius2;
        return this->MOI(L);
      }

      /// \brief Square root of positive numbers, otherwise zero.
      /// \param[in] _x Number to be square rooted.
      /// \return sqrt(_x) if _x > 0, otherwise 0
      private: static inline T ClampedSqrt(const T &_x)
      {
        if (_x <= 0)
          return 0;
        return sqrt(_x);
      }

      /// \brief Angle formed by direction of a Vector2.
      /// \param[in] _v Vector whose direction is to be computed.
      /// \param[in] _eps Minimum length of vector required for computing angle.
      /// \return Angle formed between vector and X axis,
      /// or zero if vector has length less than 1e-6.
      private: static T Angle2(const Vector2<T> &_v, const T _eps = 1e-6)
      {
        if (_v.SquaredLength() < std::pow(_eps, 2))
          return 0;
        return atan2(_v[1], _v[0]);
      }

      /// \brief Mass of the object. Default is 0.0.
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
