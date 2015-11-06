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

#include <string>

#include "ignition/math/Quaternion.hh"
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
      public: MassMatrix3() : mass(1), principals(1, 1, 1), products(0, 0, 0)
      {}

      /// \brief Constructor.
      /// \param[in] _mass Mass value in kg if using metric.
      /// \param[in] _principalMoments Principal moments of inertia
      /// \param[in] _productMoments Product moments of inertia
      public: MassMatrix3(const T &_mass,
                          const Vector3<T> &_principalMoments,
                          const Vector3<T> &_productMoments)
      : mass(_mass), principals(_principalMoments), products(_productMoments)
      {}

      /// \brief Copy constructor.
      /// \param[in] _massMatrix MassMatrix3 element to copy
      public: MassMatrix3(const MassMatrix3<T> &_m)
      : mass(_m.Mass()), principals(_m.PrincipalMoments()),
        products(_m.ProductsofInertia())
      {}

      /// \brief Destructor.
      public: virtual ~MassMatrix3() {}

      /// \brief Set the mass.
      /// \param[in] _m New mass value.
      /// \return True if the mass was set successfully.
      public: bool Mass(const T &_m)
      {
        // Should we only accept positive values?
        this->mass = _m;
        return true;
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
      /// \return True if the inertia matrix was set successfully.
      public: bool InertiaMatrix(const T &_ixx, const T &_iyy, const T &_izz,
                                 const T &_ixy, const T &_ixz, const T &_iyz)
      {
        // Should we validate the values?
        // matrix must be positive definite
        this->principals.Set(_ixx, _iyy, _izz);
        this->products.Set(_ixy, _ixz, _iyz);
        return true;
      }

      /// \brief Get the principal moments of inertia (Ixx, Iyy, Izz).
      /// \return The principal moments.
      public: Vector3<T> PrincipalMoments() const
      {
        return this->principals;
      }

      /// \brief Get the products of inertia (Ixy, Ixz, Iyz).
      /// \return The products of inertia.
      public: Vector3<T> ProductsofInertia() const
      {
        return this->products;
      }

      /// \brief Get the principal moments of inertia (Ixx, Iyy, Izz).
      /// \return The principal moments.
      public: bool PrincipalMoments(const Vector3<T> &_ixxyyzz)
      {
        // Should we validate?
        this->principals = _ixxyyzz;
        return true;
      }

      /// \brief Get the products of inertia (Ixy, Ixz, Iyz).
      /// \return The products of inertia.
      public: bool ProductsofInertia(const Vector3<T> &_ixyxzyz)
      {
        // Should we validate?
        this->products = _ixyxzyz;
        return true;
      }

      /// \brief Get IXX
      /// \return IXX value
      public: T IXX() const
      {
        return this->principals[0];
      }

      /// \brief Get IYY
      /// \return IYY value
      public: T IYY() const
      {
        return this->principals[1];
      }

      /// \brief Get IZZ
      /// \return IZZ value
      public: T IZZ() const
      {
        return this->principals[2];
      }

      /// \brief Get IXY
      /// \return IXY value
      public: T IXY() const
      {
        return this->products[0];
      }

      /// \brief Get IXZ
      /// \return IXZ value
      public: T IXZ() const
      {
        return this->products[1];
      }

      /// \brief Get IXZ
      /// \return IYZ value
      public: T IYZ() const
      {
        return this->products[2];
      }

      /// \brief Set IXX
      /// \param[in] _v IXX value
      /// \return True if the value was set successfully.
      public: bool IXX(const T &_v)
      {
        // Should we validate?
        this->principals.X(_v);
        return true;
      }

      /// \brief Set IYY
      /// \param[in] _v IYY value
      /// \return True if the value was set successfully.
      public: bool IYY(const T &_v)
      {
        // Should we validate?
        this->principals.Y(_v);
        return true;
      }

      /// \brief Set IZZ
      /// \param[in] _v IZZ value
      /// \return True if the value was set successfully.
      public: bool IZZ(const T &_v)
      {
        // Should we validate?
        this->principals.Z(_v);
        return true;
      }

      /// \brief Set IXY
      /// \param[in] _v IXY value
      /// \return True if the value was set successfully.
      public: bool IXY(const T &_v)
      {
        // Should we validate?
        this->products.X(_v);
        return true;
      }

      /// \brief Set IXZ
      /// \param[in] _v IXZ value
      /// \return True if the value was set successfully.
      public: bool IXZ(const T &_v)
      {
        // Should we validate?
        this->products.Y(_v);
        return true;
      }

      /// \brief Set IYZ
      /// \param[in] _v IXX value
      /// \return True if the value was set successfully.
      public: bool IYZ(const T &_v)
      {
        // Should we validate?
        this->products.Z(_v);
        return true;
      }

      /// \brief returns Moments of Inertia as a Matrix3
      /// \return Moments of Inertia as a Matrix3
      public: Matrix3<T> MOI() const
      {
        return Matrix3<T>(
          this->principals[0], this->products[0], this->products[1],
          this->products[0], this->principals[1], this->products[2],
          this->products[1], this->products[2], this->principals[2]);
      }

      /// \brief Sets Moments of Inertia (MOI) from a Matrix3.
      /// Symmetric component of input matrix is used by averaging
      /// off-axis terms.
      /// \param[in] Moments of Inertia as a Matrix3
      /// \return True if the inertia matrix was set successfully.
      public: bool MOI(const Matrix3<T> &_moi)
      {
        // Should we validate?
        this->principals.Set(_moi(0, 0), _moi(1, 1), _moi(2, 2));
        this->products.Set(
          0.5*(_moi(0, 1) + _moi(1, 0)),
          0.5*(_moi(0, 2) + _moi(2, 0)),
          0.5*(_moi(1, 2) + _moi(2, 1)));
        return true;
      }

      /// \brief Equal operator.
      /// \param[in] _massMatrix MassMatrix3 to copy.
      /// \return Reference to this object.
      public: MassMatrix3 &operator=(const MassMatrix3<T> &_massMatrix)
      {
        this->mass = _massMatrix.Mass();
        this->principals = _massMatrix.PrincipalMoments();
        this->products = _massMatrix.ProductsofInertia();

        return *this;
      }

      /// \brief Equality comparison operator.
      /// \param[in] _m MassMatrix3 to copy.
      /// \return true if each component is equal within a default tolerance,
      /// false otherwise
      public: bool operator==(const MassMatrix3<T> &_m) const
      {
        return equal<T>(this->mass, _m.Mass()) &&
               (this->principals == _m.PrincipalMoments()) &&
               (this->products == _m.ProductsofInertia());
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
               (this->IXX()*this->IYY() - this->IXY()*this->IXY() > 0) &&
               (this->MOI().Determinant() > 0);
      }

      /// \brief Compute Eigenvalues of Moment of Inertia Matrix.
      /// \return Eigenvalues of moment of inertia matrix.
      public: Vector3<T> EigenMoments() const
      {
        if ((this->IXY() == 0) && (this->IXZ() == 0) && (this->IYZ() == 0))
          return this->principals;

        // Algorithm based on http://arxiv.org/abs/1306.6291v4
        // A Method for Fast Diagonalization of a 2x2 or 3x3 Real Symmetric
        // Matrix, by Maarten Kronenburg
        Vector3<T> Id(this->principals);
        Vector3<T> Ip(this->products);
        // b = Ixx + Iyy + Izz
        T b = Id.Sum();
        // c = Ixx*Iyy - Ixy^2  +  Ixx*Izz - Ixz^2  +  Iyy*Izz - Iyz^2
        T c = Id[0]*Id[1] - pow(Ip[0], 2)
            + Id[0]*Id[2] - pow(Ip[1], 2)
            + Id[1]*Id[2] - pow(Ip[2], 2);
        // d = Ixx*Iyz^2 + Iyy*Ixz^2 + Izz*Ixy^2 - Ixx*Iyy*Izz - 2*Ixy*Ixz*Iyz
        T d = Id[0]*pow(Ip[2], 2) + Id[1]*pow(Ip[1], 2) + Id[2]*pow(Ip[0], 2)
            - Id[0]*Id[1]*Id[2] - 2*Ip[0]*Ip[1]*Ip[2];
        T p = pow(b, 2) - 3*c;
        T q = 2*pow(b, 3) - 9*b*c - 27*d;

        // not finished yet
        return Vector3d();
      }

      /// \brief Get dimensions and rotation offset of uniform box
      /// with equivalent mass and moment of inertia.
      /// To compute this, the Matrix3 is diagonalized.
      /// The eigenvalues on the diagonal and the rotation offset
      /// of the principal axes are returned.
      /// \param[in] _size Dimensions of box aligned with principal axes.
      /// \param[in] _rot Rotational offset of principal axes.
      /// \return True if box properties were computed successfully.
      public: bool EquivalentBox(Vector3<T> &_size, Quaternion<T> &_rot) const
      {
        return true;
      }

      /// \brief Mass the object. Default is 1.0.
      private: T mass;

      /// \brief Principal moments of inertia. Default is (1.0 1.0 1.0)
      /// These Moments of Inertia are specified in the local frame.
      private: Vector3<T> principals;

      /// \brief Product moments of inertia. Default is (0.0 0.0 0.0)
      /// These MOI off-diagonals are specified in the local frame.
      /// Where products.x is Ixy, products.y is Ixz and products.z is Iyz.
      private: Vector3<T> products;
    };

    typedef MassMatrix3<double> MassMatrix3d;
    typedef MassMatrix3<float> MassMatrix3f;
  }
}
#endif
