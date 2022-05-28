/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

%module gaussMarkovProcess
%{
#include <gz/math/GaussMarkovProcess.hh>
%}

namespace gz
{
  namespace math
  {
    class GaussMarkovProcess
    {
      public: GaussMarkovProcess();
      public: GaussMarkovProcess(double _start, double _theta, double _mu,
                  double _sigma);
      public: ~GaussMarkovProcess();
      public: void Set(double _start, double _theta, double _mu, double _sigma);
      public: double Start() const;
      public: double Value() const;
      public: double Theta() const;
      public: double Mu() const;
      public: double Sigma() const;
      public: void Reset();
      public: double Update(double _dt);
    };
  }
}
