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

/// \brief External hook to get a list of available models.
extern "C" void cmdParametersList();

/// \brief External hook to dump a parameter.
/// \param[in] _paramName Parameter name.
/// \param[in] _pose --pose option.
extern "C" void cmdParameterGet(const char *_paramName);

/// \brief External hook to set a parameter.
/// \param[in] _paramName Parameter name.
/// \param[in] _paramType Parameter type (i.e. type of the associated component).
/// \param[in] _paramValue String representation of the parameter value.
extern "C" void cmdParameterSet(
    const char *_paramName, const char * _paramType, const char *_paramValue);
