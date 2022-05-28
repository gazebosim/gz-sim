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

#include <gz/math/GaussMarkovProcess.hh>
#include <gz/math/Rand.hh>

using namespace gz::math;

//////////////////////////////////////////////////
class gz::math::GaussMarkovProcess::Implementation
{
  /// \brief Current process value.
  public: double value{0};

  /// \brief Process start value.
  public: double start{0};

  /// \brief Process theta value.
  public: double theta{0};

  /// \brief Process mu value.
  public: double mu{0};

  /// \brief Process sigma value.
  public: double sigma{0};
};

//////////////////////////////////////////////////
GaussMarkovProcess::GaussMarkovProcess()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
GaussMarkovProcess::GaussMarkovProcess(double _start, double _theta, double _mu,
    double _sigma)
  : GaussMarkovProcess()
{
  this->Set(_start, _theta, _mu, _sigma);
}

//////////////////////////////////////////////////
void GaussMarkovProcess::Set(double _start, double _theta, double _mu,
    double _sigma)
{
  this->dataPtr->start = _start;
  this->dataPtr->theta = std::max(0.0, _theta);
  this->dataPtr->mu = _mu;
  this->dataPtr->sigma = std::max(0.0, _sigma);
  this->Reset();
}

//////////////////////////////////////////////////
void GaussMarkovProcess::Reset()
{
  this->dataPtr->value = this->dataPtr->start;
}

//////////////////////////////////////////////////
double GaussMarkovProcess::Start() const
{
  return this->dataPtr->start;
}

//////////////////////////////////////////////////
double GaussMarkovProcess::Value() const
{
  return this->dataPtr->value;
}

//////////////////////////////////////////////////
double GaussMarkovProcess::Theta() const
{
  return this->dataPtr->theta;
}

//////////////////////////////////////////////////
double GaussMarkovProcess::Mu() const
{
  return this->dataPtr->mu;
}

//////////////////////////////////////////////////
double GaussMarkovProcess::Sigma() const
{
  return this->dataPtr->sigma;
}

//////////////////////////////////////////////////
double GaussMarkovProcess::Update(const clock::duration &_dt)
{
  // Time difference in seconds
  return this->Update(std::chrono::duration<double>(_dt).count());
}

//////////////////////////////////////////////////
double GaussMarkovProcess::Update(double _dt)
{
  this->dataPtr->value += this->dataPtr->theta *
    (this->dataPtr->mu - this->dataPtr->value) * _dt +
    this->dataPtr->sigma * Rand::DblNormal(0, 1);

  // Output the new value.
  return this->dataPtr->value;
}
