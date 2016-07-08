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
#ifndef IGNITION_MATH_PID_HH_
#define IGNITION_MATH_PID_HH_

#include <chrono>
#include <ignition/math/Helpers.hh>

namespace ignition
{
  namespace math
  {
    /// \class PID PID.hh ignition/math/PID.hh
    /// \brief Generic PID controller class.
    /// Generic proportiolnal-integral-derivative controller class that
    /// keeps track of PID-error states and control inputs given
    /// the state of a system and a user specified target state.
    class IGNITION_VISIBLE PID
    {
      /// \brief Constructor, zeros out Pid values when created and
      /// initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
      ///
      /// Disable command clamping by setting _cmdMin to a value larger
      /// than _cmdMax. Command clamping is disabled by default.
      ///
      /// \param[in] _p  The proportional gain.
      /// \param[in] _i  The integral gain.
      /// \param[in] _d  The derivative gain.
      /// \param[in] _imax The integral upper limit.
      /// \param[in] _imin The integral lower limit.
      /// \param[in] _cmdMax Output max value.
      /// \param[in] _cmdMin Output min value.
      public: PID(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                  double _imax = 0.0, double _imin = 0.0,
                  double _cmdMax = -1.0, double _cmdMin = 0.0);

      /// \brief Destructor
      public: ~PID() = default;

      /// \brief Initialize PID-gains and integral term
      ///        limits:[iMax:iMin]-[I1:I2].
      ///
      /// Disable command clamping by setting _cmdMin to a value larger
      /// than _cmdMax. Command clamping is disabled by default.
      ///
      /// \param[in] _p  The proportional gain.
      /// \param[in] _i  The integral gain.
      /// \param[in] _d  The derivative gain.
      /// \param[in] _imax The integral upper limit.
      /// \param[in] _imin The integral lower limit.
      /// \param[in] _cmdMax Output max value.
      /// \param[in] _cmdMin Output min value.
      public: void Init(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                        double _imax = 0.0, double _imin = 0.0,
                        double _cmdMax = -1.0, double _cmdMin = 0.0);

      /// \brief Set the proportional Gain.
      /// \param[in] _p proportional gain value
      public: void SetPGain(double _p);

      /// \brief Set the integral Gain.
      /// \param[in] _p integral gain value
      public: void SetIGain(double _i);

      /// \brief Set the derivtive Gain.
      /// \param[in] _p dertivative gain value
      public: void SetDGain(double _d);

      /// \brief Set the integral upper limit.
      /// \param[in] _p integral upper limit value
      public: void SetIMax(double _i);

      /// \brief Set the integral lower limit.
      /// \param[in] _p integral lower limit value
      public: void SetIMin(double _i);

      /// \brief Set the maximum value for the command.
      /// \param[in] _c The maximum value
      public: void SetCmdMax(double _c);

      /// \brief Set the maximum value for the command.
      /// \param[in] _c The maximum value
      public: void SetCmdMin(double _c);

      /// \brief Get the proportional Gain.
      /// \return The proportional gain value
      public: double PGain() const;

      /// \brief Get the integral Gain.
      /// \return The integral gain value
      public: double IGain() const;

      /// \brief Get the derivative Gain.
      /// \return The derivative gain value
      public: double DGain() const;

      /// \brief Get the integral upper limit.
      /// \return The integral upper limit value
      public: double IMax() const;

      /// \brief Get the integral lower limit.
      /// \return The integral lower limit value
      public: double IMin() const;

      /// \brief Get the maximum value for the command.
      /// \return The maximum value
      public: double CmdMax() const;

      /// \brief Get the maximum value for the command.
      /// \return The maximum value
      public: double CmdMin() const;

      /// \brief Update the Pid loop with nonuniform time step size.
      /// \param[_in] _error  Error since last call (p_state - p_target).
      /// \param[_in] _dt Change in time since last update call.
      /// Normally, this is called at every time step,
      /// The return value is an updated command to be passed
      /// to the object being controlled.
      /// \return the command value
      public: double Update(double _error, std::chrono::duration<double> _dt);

      /// \brief Set current target command for this PID controller.
      /// \param[in] _cmd New command
      public: void SetCmd(double _cmd);

      /// \brief Return current command for this PID controller.
      /// \return the command value
      public: double Cmd() const;

      /// \brief Return PID error terms for the controller.
      /// \param[in] _pe  The proportional error.
      /// \param[in] _ie  The integral error.
      /// \param[in] _de  The derivative error.
      public: void Errors(double &_pe, double &_ie, double &_de) const;

      /// \brief Assignment operator
      /// \param[in] _p a reference to a PID to assign values from
      /// \return reference to this instance
      public: PID &operator=(const PID &_p);

      /// \brief Reset the errors and command.
      public: void Reset();

      /// \brief Error at a previous step.
      private: double pErrLast = 0.0;

      /// \brief Current error.
      private: double pErr = 0.0;

      /// \brief Integral error.
      private: double iErr = 0.0;

      /// \brief Derivative error.
      private: double dErr = 0.0;

      /// \brief Gain for proportional control.
      private: double pGain;

      /// \brief Gain for integral control.
      private: double iGain = 0.0;

      /// \brief Gain for derivative control.
      private: double dGain = 0.0;

      /// \brief Maximum clamping value for integral term.
      private: double iMax = 0.0;

      /// \brief Minim clamping value for integral term.
      private: double iMin = 0.0;

      /// \brief Command value.
      private: double cmd = 0.0;

      /// \brief Max command clamping value.
      private: double cmdMax = -1.0;

      /// \brief Min command clamping value.
      private: double cmdMin = 0.0;
    };
  }
}
#endif
