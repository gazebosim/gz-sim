/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_HYDRODYNAMICS_UTILS_HH_
#define GZ_SIM_SYSTEMS_HYDRODYNAMICS_UTILS_HH_

#include <cmath>

#include <Eigen/Eigen>

namespace gz
{
namespace sim
{
namespace systems
{
namespace hydrodynamics
{
  /// \brief Build the negated added-mass Coriolis matrix (-C_A) per
  /// Fossen (2011) Theorem 6.2, eq 6.43.
  ///
  /// Uses only diagonal elements of _Ma. The returned matrix is
  /// skew-symmetric: C + C^T = 0. The Fossen sign convention is that
  /// diagonal elements of _Ma are negative (e.g. X_u_dot < 0).
  ///
  /// \param[in] _Ma 6x6 added mass matrix
  /// \param[in] _state 6x1 body-frame velocity [u,v,w,p,q,r]
  /// \return 6x6 matrix equal to -C_A (skew-symmetric)
  inline Eigen::Matrix<double, 6, 6> buildCoriolisMatrix(
    const Eigen::Matrix<double, 6, 6> &_Ma,
    const Eigen::Matrix<double, 6, 1> &_state)
  {
    Eigen::Matrix<double, 6, 6> Cmat =
      Eigen::Matrix<double, 6, 6>::Zero();

    Cmat(0, 4) = - _Ma(2, 2) * _state(2);
    Cmat(0, 5) =   _Ma(1, 1) * _state(1);
    Cmat(1, 3) =   _Ma(2, 2) * _state(2);
    Cmat(1, 5) = - _Ma(0, 0) * _state(0);
    Cmat(2, 3) = - _Ma(1, 1) * _state(1);
    Cmat(2, 4) =   _Ma(0, 0) * _state(0);
    Cmat(3, 1) = - _Ma(2, 2) * _state(2);
    Cmat(3, 2) =   _Ma(1, 1) * _state(1);
    Cmat(3, 4) = - _Ma(5, 5) * _state(5);
    Cmat(3, 5) =   _Ma(4, 4) * _state(4);
    Cmat(4, 0) =   _Ma(2, 2) * _state(2);
    Cmat(4, 2) = - _Ma(0, 0) * _state(0);
    Cmat(4, 3) =   _Ma(5, 5) * _state(5);
    Cmat(4, 5) = - _Ma(3, 3) * _state(3);
    Cmat(5, 0) = - _Ma(1, 1) * _state(1);
    Cmat(5, 1) =   _Ma(0, 0) * _state(0);
    Cmat(5, 3) = - _Ma(4, 4) * _state(4);
    Cmat(5, 4) =   _Ma(3, 3) * _state(3);

    return Cmat;
  }

  /// \brief Build the hydrodynamic damping matrix from linear and
  /// quadratic stability derivatives.
  ///
  /// For each (i,j) entry the coefficient is:
  ///   D(i,j) = -linear[i*6+j]
  ///            - sum_k( quadAbs[i*36+j*6+k] * |state(k)| )
  ///            - sum_k( quad[i*36+j*6+k] * state(k) )
  ///
  /// The damping force is then D * state.
  ///
  /// \param[in] _linearTerms 36-element array of linear damping coefficients
  /// \param[in] _quadAbsDerivs 216-element array of |v|-quadratic damping
  /// \param[in] _quadDerivs 216-element array of v-quadratic damping
  /// \param[in] _state 6x1 body-frame velocity [u,v,w,p,q,r]
  /// \return 6x6 damping matrix D
  inline Eigen::Matrix<double, 6, 6> buildDampingMatrix(
    const double (&_linearTerms)[36],
    const double (&_quadAbsDerivs)[216],
    const double (&_quadDerivs)[216],
    const Eigen::Matrix<double, 6, 1> &_state)
  {
    Eigen::Matrix<double, 6, 6> Dmat =
      Eigen::Matrix<double, 6, 6>::Zero();

    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        auto coeff = - _linearTerms[i * 6 + j];
        for (int k = 0; k < 6; k++)
        {
          auto index = i * 36 + j * 6 + k;
          coeff -= _quadAbsDerivs[index] * std::abs(_state(k));
          coeff -= _quadDerivs[index] * _state(k);
        }
        Dmat(i, j) = coeff;
      }
    }

    return Dmat;
  }

  /// \brief 3x3 skew-symmetric (cross-product) matrix from a 3-vector.
  ///
  /// S(v) * x = v x x  (cross product)
  ///
  /// \param[in] _v 3-vector
  /// \return 3x3 skew-symmetric matrix
  inline Eigen::Matrix3d skew3(const Eigen::Vector3d &_v)
  {
    Eigen::Matrix3d S;
    S <<       0, -_v(2),  _v(1),
          _v(2),       0, -_v(0),
         -_v(1),  _v(0),       0;
    return S;
  }

  /// \brief Build the added-mass Coriolis matrix C_A(v) for a full
  /// (non-diagonal) 6x6 added mass matrix, per Fossen (2011) eq. 6.43.
  ///
  /// This function uses the physical (positive) sign convention for M_A,
  /// matching the SDF `<fluid_added_mass>` specification.
  ///
  /// Given:
  ///   M_A = | M11  M12 |    v = | v1 |   (v1 = linear, v2 = angular)
  ///         | M21  M22 |        | v2 |
  ///
  /// The Coriolis matrix is:
  ///   C_A(v) = |       0          -S(M11*v1 + M12*v2) |
  ///            | -S(M11*v1+M12*v2) -S(M21*v1 + M22*v2) |
  ///
  /// The returned matrix is skew-symmetric: C_A + C_A^T = 0.
  ///
  /// \param[in] _Ma 6x6 added mass matrix (positive physical values)
  /// \param[in] _state 6x1 body-frame velocity [u,v,w,p,q,r]
  /// \return 6x6 Coriolis matrix C_A (skew-symmetric)
  inline Eigen::Matrix<double, 6, 6> buildFullCoriolisMatrix(
    const Eigen::Matrix<double, 6, 6> &_Ma,
    const Eigen::Matrix<double, 6, 1> &_state)
  {
    Eigen::Vector3d v1 = _state.head<3>();
    Eigen::Vector3d v2 = _state.tail<3>();

    Eigen::Matrix3d M11 = _Ma.block<3, 3>(0, 0);
    Eigen::Matrix3d M12 = _Ma.block<3, 3>(0, 3);
    Eigen::Matrix3d M21 = _Ma.block<3, 3>(3, 0);
    Eigen::Matrix3d M22 = _Ma.block<3, 3>(3, 3);

    Eigen::Vector3d a1 = M11 * v1 + M12 * v2;
    Eigen::Vector3d a2 = M21 * v1 + M22 * v2;

    Eigen::Matrix<double, 6, 6> C =
      Eigen::Matrix<double, 6, 6>::Zero();
    C.block<3, 3>(0, 3) = -skew3(a1);
    C.block<3, 3>(3, 0) = -skew3(a1);
    C.block<3, 3>(3, 3) = -skew3(a2);

    return C;
  }

}  // namespace hydrodynamics
}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif
