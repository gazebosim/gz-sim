/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <string>

#include "Matrix4.hh"

namespace gz
{
namespace math
{
namespace python
{
void defineMathMatrix4(py::module &m, const std::string &typestr)
{
  helpDefineMathMatrix4<double>(m, typestr + "d");
  helpDefineMathMatrix4<float>(m, typestr + "f");
  helpDefineMathMatrix4<int>(m, typestr + "i");
}

}  // namespace python
}  // namespace math
}  // namespace gz
