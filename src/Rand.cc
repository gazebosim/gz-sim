/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: Random number generator
 * Author: Nate Koenig
 * Date: 27 May 2009
 */

#include <sys/types.h>
#include <unistd.h>
#include <ctime>

#include "ignition/math/Rand.hh"

using namespace ignition;
using namespace math;


// We don't seed with time for the cases when two processes are started the
// same time (this mostly happens with launch scripts that start a server
// and gui simultaneously).
uint32_t Rand::seed = getpid();

GeneratorType *Rand::randGenerator = new GeneratorType(seed);

//////////////////////////////////////////////////
void Rand::SetSeed(uint32_t _seed)
{
  seed = _seed;
  randGenerator->seed(seed);
}

//////////////////////////////////////////////////
uint32_t Rand::GetSeed()
{
  return seed;
}

//////////////////////////////////////////////////
double Rand::GetDblUniform(double _min, double _max)
{
  URealGen gen(*randGenerator, UniformRealDist(_min, _max));

  return gen();
}

//////////////////////////////////////////////////
double Rand::GetDblNormal(double _mean, double _sigma)
{
  NRealGen gen(*randGenerator, NormalRealDist(_mean, _sigma));

  return gen();
}

//////////////////////////////////////////////////
int Rand::GetIntUniform(int _min, int _max)
{
  UIntGen gen(*randGenerator, UniformIntDist(_min, _max));

  return gen();
}

//////////////////////////////////////////////////
int Rand::GetIntNormal(int _mean, int _sigma)
{
  NRealGen gen(*randGenerator, NormalRealDist(_mean, _sigma));

  return static_cast<int>(gen());
}

