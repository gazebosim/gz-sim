/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_INERTIAL_HH_
#define IGNITION_MATH_INERTIAL_HH_

#include "ignition/math/MassMatrix3.hh"
#include "ignition/math/Pose3.hh"

namespace ignition
{
  namespace math
  {
    /// \class Inertial Inertial.hh ignition/math/Inertial.hh
    /// \brief A class for inertial information about a rigid body
    /// consisting of the scalar mass, a 3x3 symmetric moment
    /// of inertia matrix, and center of mass reference frame pose.
    template<typename T>
    class Inertial
    {
      /// \brief Default Constructor
      public: Inertial()
      {}

      /// \brief Constructor.
      /// \param[in] _massMatrix Mass and inertia matrix.
      /// \param[in] _pose Pose of center of mass reference frame.
      public: Inertial(const MassMatrix3<T> &_massMatrix,
                       const Pose3<T> &_pose)
      : massMatrix(_massMatrix), pose(_pose)
      {}

      /// \brief Copy constructor.
      /// \param[in] _inertial Inertial element to copy
      public: Inertial(const Inertial<T> &_inertial)
      : massMatrix(_inertial.MassMatrix()), pose(_inertial.Pose())
      {}

      /// \brief Destructor.
      public: virtual ~Inertial() {}

      /// \brief Set the mass and inertia matrix.
      /// \param[in] _m New MassMatrix3 object.
      /// \return True if the MassMatrix3 is valid.
      public: bool SetMassMatrix(const MassMatrix3<T> &_m)
      {
        this->massMatrix = _m;
        return this->massMatrix.IsValid();
      }

      /// \brief Get the mass and inertia matrix.
      /// \return The MassMatrix3 object.
      public: const MassMatrix3<T> &MassMatrix() const
      {
        return this->massMatrix;
      }

      /// \brief Set the pose of center of mass reference frame.
      /// \param[in] _pose New pose.
      /// \return True if the MassMatrix3 is valid.
      public: bool SetPose(const Pose3<T> &_pose)
      {
        this->pose = _pose;
        return this->massMatrix.IsValid();
      }

      /// \brief Get the pose of center of mass reference frame.
      /// \return The pose of center of mass reference frame.
      public: const Pose3<T> &Pose() const
      {
        return this->pose;
      }

      /// \brief Get the moment of inertia matrix expressed in the
      /// base coordinate frame.
      /// \return Rotated moment of inertia matrix.
      public: Matrix3<T> MOI() const
      {
        auto R = Matrix3<T>(this->pose.Rot());
        return R * this->massMatrix.MOI() * R.Transposed();
      }

      /// \brief Set the inertial pose rotation without affecting the
      /// MOI in the base coordinate frame.
      /// \param[in] _q New rotation for inertial pose.
      /// \return True if the MassMatrix3 is valid.
      public: bool SetInertialRotation(const Quaternion<T> &_q)
      {
        auto moi = this->MOI();
        this->pose.Rot() = _q;
        auto R = Matrix3<T>(_q);
        return this->massMatrix.MOI(R.Transposed() * moi * R);
      }

      /// \brief Set the MassMatrix rotation (eigenvectors of inertia matrix)
      /// without affecting the MOI in the base coordinate frame.
      /// Note that symmetries in inertia matrix may prevent the output of
      /// MassMatrix3::PrincipalAxesOffset to match this function's input _q,
      /// but it is guaranteed that the MOI in the base frame will not change.
      /// A negative value of _tol (such as -1e-6) can be passed to ensure
      /// that diagonal values are always sorted.
      /// \param[in] _q New rotation.
      /// \param[in] _tol Relative tolerance given by absolute value
      /// of _tol. This is passed to the MassMatrix3
      /// PrincipalMoments and PrincipalAxesOffset functions.
      /// \return True if the MassMatrix3 is valid.
      public: bool SetMassMatrixRotation(const Quaternion<T> &_q,
                                         const T _tol = 1e-6)
      {
        this->pose.Rot() *= this->MassMatrix().PrincipalAxesOffset(_tol) *
                            _q.Inverse();
        const auto moments = this->MassMatrix().PrincipalMoments(_tol);
        const auto diag = Matrix3<T>(
            moments[0], 0, 0,
            0, moments[1], 0,
            0, 0, moments[2]);
        const auto R = Matrix3<T>(_q);
        return this->massMatrix.MOI(R * diag * R.Transposed());
      }

      /// \brief Equal operator.
      /// \param[in] _inertial Inertial to copy.
      /// \return Reference to this object.
      public: Inertial &operator=(const Inertial<T> &_inertial)
      {
        this->massMatrix = _inertial.MassMatrix();
        this->pose = _inertial.Pose();

        return *this;
      }

      /// \brief Equality comparison operator.
      /// \param[in] _inertial Inertial to copy.
      /// \return true if each component is equal within a default tolerance,
      /// false otherwise
      public: bool operator==(const Inertial<T> &_inertial) const
      {
        return (this->pose == _inertial.Pose()) &&
               (this->massMatrix == _inertial.MassMatrix());
      }

      /// \brief Inequality test operator
      /// \param[in] _inertial Inertial<T> to test
      /// \return True if not equal (using the default tolerance of 1e-6)
      public: bool operator!=(const Inertial<T> &_inertial) const
      {
        return !(*this == _inertial);
      }

      /// \brief Adds inertial properties to current object.
      /// The mass, center of mass location, and inertia matrix are updated
      /// as long as the total mass is positive.
      /// \param[in] _inertial Inertial to add.
      /// \return Reference to this object.
      public: Inertial<T> &operator+=(const Inertial<T> &_inertial)
      {
        T m1 = this->massMatrix.Mass();
        T m2 = _inertial.MassMatrix().Mass();

        // Total mass
        T mass = m1 + m2;

        // Only continue if total mass is positive
        if (mass <= 0)
        {
          return *this;
        }

        auto com1 = this->Pose().Pos();
        auto com2 = _inertial.Pose().Pos();
        // New center of mass location in base frame
        auto com = (m1*com1 + m2*com2) / mass;

        // Components of new moment of inertia matrix
        Vector3<T> ixxyyzz;
        Vector3<T> ixyxzyz;
        // First add matrices in base frame
        {
          auto moi = this->MOI() + _inertial.MOI();
          ixxyyzz = Vector3<T>(moi(0, 0), moi(1, 1), moi(2, 2));
          ixyxzyz = Vector3<T>(moi(0, 1), moi(0, 2), moi(1, 2));
        }
        // Then account for parallel axis theorem
        {
          auto dc = com1 - com;
          ixxyyzz.X() += m1 * (std::pow(dc[1], 2) + std::pow(dc[2], 2));
          ixxyyzz.Y() += m1 * (std::pow(dc[2], 2) + std::pow(dc[0], 2));
          ixxyyzz.Z() += m1 * (std::pow(dc[0], 2) + std::pow(dc[1], 2));
          ixxyyzz.X() -= m1 * dc[0] * dc[1];
          ixxyyzz.Y() -= m1 * dc[0] * dc[2];
          ixxyyzz.Z() -= m1 * dc[1] * dc[2];
        }
        {
          auto dc = com2 - com;
          ixxyyzz.X() += m2 * (std::pow(dc[1], 2) + std::pow(dc[2], 2));
          ixxyyzz.Y() += m2 * (std::pow(dc[2], 2) + std::pow(dc[0], 2));
          ixxyyzz.Z() += m2 * (std::pow(dc[0], 2) + std::pow(dc[1], 2));
          ixxyyzz.X() -= m2 * dc[0] * dc[1];
          ixxyyzz.Y() -= m2 * dc[0] * dc[2];
          ixxyyzz.Z() -= m2 * dc[1] * dc[2];
        }
        this->massMatrix = MassMatrix3<T>(mass, ixxyyzz, ixyxzyz);
        this->pose = Pose3<T>(com, Quaternion<T>::Identity);

        return *this;
      }

      /// \brief Adds inertial properties to current object.
      /// The mass, center of mass location, and inertia matrix are updated
      /// as long as the total mass is positive.
      /// \param[in] _inertial Inertial to add.
      /// \return Sum of inertials as new object.
      public: const Inertial<T> operator+(const Inertial<T> &_inertial) const
      {
        return Inertial<T>(*this) += _inertial;
      }

      /// \brief Mass and inertia matrix of the object expressed in the
      /// center of mass reference frame.
      private: MassMatrix3<T> massMatrix;

      /// \brief Pose offset of center of mass reference frame relative
      /// to a base frame.
      private: Pose3<T> pose;
    };

    typedef Inertial<double> Inertiald;
    typedef Inertial<float> Inertialf;
  }
}
#endif
