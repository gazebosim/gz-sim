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
//! [complete]
#include <stdio.h>
#include <gz/math/Temperature.hh>

int main(int argc, char **argv)
{
  /// Convert from Kelvin to Celsius
  double celsius = gz::math::Temperature::KelvinToCelsius(2.5);
  printf("2.5Kelvin to Celsius is %f\n", celsius);

  gz::math::Temperature temp(123.5);
  printf("Constructed a Temperature object with %f Kelvin\n", temp());
  printf("Same temperature in Celsius %f\n", temp.Celsius());

  temp += 100.0;
  printf("Temperature + 100.0 is %fK\n", temp());

  gz::math::Temperature newTemp(temp);
  newTemp += temp + 23.5;
  printf("Copied the temp object and added 23.5K. The new tempurature is %fF\n",
      newTemp.Fahrenheit());

  return 0;
}
//! [complete]
