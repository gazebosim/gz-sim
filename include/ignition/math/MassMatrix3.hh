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
      public: MassMatrix3();

      /// \brief Constructor.
      /// \param[in] _mass Mass value in kg if using metric.
      /// \param[in] _principalMoments Principal moments of inertia
      /// \param[in] _productMoments Product moments of inertia
      public: explicit MassMatrix3(const T &_mass,
                                   const Vector3<T> &_principalMoments,
                                   const Vector3<T> &_productMoments);

      /// \brief Copy constructor.
      /// \param[in] _massMatrix MassMatrix3 element to copy
      public: MassMatrix3(const MassMatrix3<T> &_massMatrix);

      /// \brief Destructor.
      public: virtual ~MassMatrix3();

      /// \brief Set the mass.
      /// \param[in] _m New mass value.
      /// \return True if the mass was set successfully.
      public: bool Mass(const T &_m);

      /// \brief Get the mass
      /// \return The mass value
      public: T Mass() const;

      /// \brief Set the moment of inertia matrix.
      /// \param[in] _ixx X second moment of inertia (MOI) about x axis.
      /// \param[in] _iyy Y second moment of inertia about y axis.
      /// \param[in] _izz Z second moment of inertia about z axis.
      /// \param[in] _ixy XY inertia.
      /// \param[in] _ixz XZ inertia.
      /// \param[in] _iyz YZ inertia.
      /// \return True if the inertia matrix was set successfully.
      public: bool InertiaMatrix(const T &_ixx, const T &_iyy, const T &_izz,
                                 const T &_ixy, const T &_ixz, const T &_iyz);

      /// \brief Get the principal moments of inertia (Ixx, Iyy, Izz).
      /// \return The principal moments.
      public: Vector3<T> PrincipalMoments() const;

      /// \brief Get the products of inertia (Ixy, Ixz, Iyz).
      /// \return The products of inertia.
      public: Vector3<T> ProductsofInertia() const;

      /// \brief Get IXX
      /// \return IXX value
      public: T IXX() const;

      /// \brief Get IYY
      /// \return IYY value
      public: T IYY() const;

      /// \brief Get IZZ
      /// \return IZZ value
      public: T IZZ() const;

      /// \brief Get IXY
      /// \return IXY value
      public: T IXY() const;

      /// \brief Get IXZ
      /// \return IXZ value
      public: T IXZ() const;

      /// \brief Get IXZ
      /// \return IYZ value
      public: T IYZ() const;

      /// \brief Set IXX
      /// \param[in] _v IXX value
      /// \return True if the value was set successfully.
      public: bool IXX(const T &_v);

      /// \brief Set IYY
      /// \param[in] _v IYY value
      /// \return True if the value was set successfully.
      public: bool IYY(const T &_v);

      /// \brief Set IZZ
      /// \param[in] _v IZZ value
      /// \return True if the value was set successfully.
      public: bool IZZ(const T &_v);

      /// \brief Set IXY
      /// \param[in] _v IXY value
      /// \return True if the value was set successfully.
      public: bool IXY(const T &_v);

      /// \brief Set IXZ
      /// \param[in] _v IXZ value
      /// \return True if the value was set successfully.
      public: bool IXZ(const T &_v);

      /// \brief Set IYZ
      /// \param[in] _v IXX value
      /// \return True if the value was set successfully.
      public: bool IYZ(const T &_v);

      /// \brief returns Moments of Inertia as a Matrix3
      /// \return Moments of Inertia as a Matrix3
      public: Matrix3<T> MOI() const;

      /// \brief Sets Moments of Inertia (MOI) from a Matrix3
      /// \param[in] Moments of Inertia as a Matrix3
      /// \return True if the inertia matrix was set successfully.
      public: bool MOI(const Matrix3<T> &_moi);

      /// \brief Equal operator.
      /// \param[in] _massMatrix MassMatrix3 to copy.
      /// \return Reference to this object.
      public: MassMatrix3 &operator=(const MassMatrix3<T> &_massMatrix);

      /// \brief Get dimensions and rotation offset of uniform box
      /// with equivalent mass and moment of inertia.
      /// To compute this, the Matrix3 is diagonalized.
      /// The eigenvalues on the diagonal and the rotation offset
      /// of the principal axes are returned.
      /// \param[in] _size Dimensions of box aligned with principal axes.
      /// \param[in] _rot Rotational offset of principal axes.
      /// \return True if box properties were computed successfully.
      public: bool EquivalentBox(Vector3<T> &_size, Quaternion<T> &_rot);

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
