/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
%module pid 
%{
	#include <gz/math/PID.hh>
%}

%include "typemaps.i"
%apply double *OUTPUT { double &_pe, double &_ie, double &_de };

namespace gz
{
  namespace math
  {
    class PID
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: PID(const double _p = 0.0,
                  const double _i = 0.0,
                  const double _d = 0.0,
                  const double _imax = -1.0,
                  const double _imin = 0.0,
                  const double _cmdMax = -1.0,
                  const double _cmdMin = 0.0,
                  const double _cmdOffset = 0.0);
      public: PID(const PID& pid) = default;
      public: ~PID() = default;
      public: void Init(const double _p = 0.0,
                        const double _i = 0.0,
                        const double _d = 0.0,
                        const double _imax = -1.0,
                        const double _imin = 0.0,
                        const double _cmdMax = -1.0,
                        const double _cmdMin = 0.0,
                        const double _cmdOffset = 0.0);
      %rename(set_p_gain) SetPGain;
      public: void SetPGain(const double _p);
      %rename(set_i_gain) SetIGain;
      public: void SetIGain(const double _i);
      %rename(set_d_gain) SetDGain;
      public: void SetDGain(const double _d);
      %rename(set_i_max) SetIMax;
      public: void SetIMax(const double _i);
      %rename(set_i_min) SetIMin;
      public: void SetIMin(const double _i);
      public: void SetCmdMax(const double _c);
      public: void SetCmdMin(const double _c);
      public: void SetCmdOffset(const double _c);
      %rename(p_gain) PGain;
      public: double PGain() const;
      %rename(i_gain) IGain;
      public: double IGain() const;
      %rename(d_gain) DGain;
      public: double DGain() const;
      %rename(i_max) IMax;
      public: double IMax() const;
      %rename(i_min) IMin;
      public: double IMin() const;
      public: double CmdMax() const;
      public: double CmdMin() const;
      public: double CmdOffset() const;
      public: void SetCmd(const double _cmd);
      public: double Cmd() const;
      public: void Errors(double &_pe, double &_ie, double &_de) const;
      public: void Reset();
    };

    %extend PID {
      double Update(const double error, const double dt) {
        return (*$self).Update(error, std::chrono::duration<double>(dt));
      }
    }
  }
}
