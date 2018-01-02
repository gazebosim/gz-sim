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

#include <sstream>

#include "ignition/math/SemanticVersion.hh"

using namespace ignition;
using namespace math;

namespace ignition
{
  namespace math
  {
    inline namespace IGNITION_MATH_VERSION_NAMESPACE
    {
    class SemanticVersionPrivate
    {
      /// \brief Major revision (incompatible api changes)
      public: unsigned int maj = 0;

      /// \brief Minor revision (backwards compatible new functionality)
      public: unsigned int min = 0;

      /// \brief Patch (bug fixes)
      public: unsigned int patch = 0;

      /// \brief Optional pre-release info. A prerelease string may be
      /// denoted by appending a hyphen and a series of dot separated
      /// identifiers immediately following the patch version
      public: std::string prerelease = "";

      /// \brief Optional build meta-data. Build metadata may be denoted by
      /// appending a plus sign and a series of dot separated identifiers
      /// immediately following the patch or pre-release version
      public: std::string build = "";
    };
    }
  }
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion()
: dataPtr(new SemanticVersionPrivate())
{
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const std::string &_versionStr)
: dataPtr(new SemanticVersionPrivate())
{
  this->Parse(_versionStr);
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const unsigned int _major,
  const unsigned int _minor,
  const unsigned int _patch,
  const std::string &_prerelease,
  const std::string &_build)
: dataPtr(new SemanticVersionPrivate())
{
  this->dataPtr->maj = _major;
  this->dataPtr->min = _minor;
  this->dataPtr->patch = _patch;
  this->dataPtr->prerelease = _prerelease;
  this->dataPtr->build = _build;
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const SemanticVersion  &_copy)
: dataPtr(new SemanticVersionPrivate())
{
  *this->dataPtr = *_copy.dataPtr;
}

/////////////////////////////////////////////////
SemanticVersion& SemanticVersion::operator=(const SemanticVersion &_other)
{
  *this->dataPtr = *_other.dataPtr;
  return *this;
}

/////////////////////////////////////////////////
SemanticVersion::~SemanticVersion()
{
}

/////////////////////////////////////////////////
std::string SemanticVersion::Version() const
{
  std::string result = std::to_string(this->dataPtr->maj) + "." +
    std::to_string(this->dataPtr->min) + "." +
    std::to_string(this->dataPtr->patch);

  if (!this->dataPtr->prerelease.empty())
    result += "-" + this->dataPtr->prerelease;

  if (!this->dataPtr->build.empty())
    result += "+" + this->dataPtr->build;

  return result;
}

/////////////////////////////////////////////////
bool SemanticVersion::operator<(const SemanticVersion &_other) const
{
  if (this == &_other)
    return false;

  if (this->dataPtr->maj < _other.dataPtr->maj)
  {
    return true;
  }
  if (this->dataPtr->maj > _other.dataPtr->maj)
  {
    return false;
  }
  if (this->dataPtr->min < _other.dataPtr->min)
  {
    return true;
  }
  if (this->dataPtr->min > _other.dataPtr->min)
  {
    return false;
  }
  if (this->dataPtr->patch < _other.dataPtr->patch)
  {
    return true;
  }
  if (this->dataPtr->patch > _other.dataPtr->patch)
  {
    return false;
  }

  // If this version has prelrease and the _other doesn't, then this
  // version is lower. We don't compare the prerelease strings because
  // they can contain any alphanumeric values.
  if (!this->dataPtr->prerelease.empty() && _other.dataPtr->prerelease.empty())
    return true;

  // _other is equal
  return false;
}

/////////////////////////////////////////////////
bool SemanticVersion::operator<=(const SemanticVersion &_other) const
{
  return (_other > *this) || (_other == *this);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator>(const SemanticVersion &_other) const
{
  return (_other < *this);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator>=(const SemanticVersion &_other) const
{
  return (_other < *this) || (_other == *this);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator==(const SemanticVersion &_other) const
{
  if (this == &_other)
    return true;

  return (_other.dataPtr->maj == this->dataPtr->maj)
    && (_other.dataPtr->min == this->dataPtr->min)
    && (_other.dataPtr->patch == this->dataPtr->patch);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator!=(const SemanticVersion &_other) const
{
  return !(*this == _other);
}

/////////////////////////////////////////////////
unsigned int SemanticVersion::Major() const
{
  return this->dataPtr->maj;
}

/////////////////////////////////////////////////
unsigned int SemanticVersion::Minor() const
{
  return this->dataPtr->min;
}

/////////////////////////////////////////////////
unsigned int SemanticVersion::Patch() const
{
  return this->dataPtr->patch;
}

/////////////////////////////////////////////////
std::string SemanticVersion::Prerelease() const
{
  return this->dataPtr->prerelease;
}

/////////////////////////////////////////////////
std::string SemanticVersion::Build() const
{
  return this->dataPtr->build;
}

/////////////////////////////////////////////////
bool SemanticVersion::Parse(const std::string &_versionStr)
{
  if (_versionStr.empty())
    return false;

  size_t numericEnd = _versionStr.size();
  size_t prereleaseStart = _versionStr.find("-");
  size_t buildStart = _versionStr.find("+");

  // Build meta data, if present, must be after prerelease string, if
  // present
  if (buildStart != std::string::npos &&
      prereleaseStart != std::string::npos &&
      buildStart < prereleaseStart)
  {
    return false;
  }

  // Check if a prerelease version is present
  if (prereleaseStart != std::string::npos)
  {
    // Set the end of the numeric (major.minor.patch) portion to the
    // start of the prerelease
    numericEnd = prereleaseStart;

    // Check if build metadata is present
    if (buildStart != std::string::npos)
    {
      // Set prerelease to the portion between "-" and "+"
      this->dataPtr->prerelease = _versionStr.substr(numericEnd + 1,
           buildStart - numericEnd - 1);

      // Get the build metadata.
      this->dataPtr->build = _versionStr.substr(buildStart + 1);
    }
    else
    {
      // Set prerelease to the portion between "-" and the string's end
      this->dataPtr->prerelease = _versionStr.substr(numericEnd + 1);
    }
  }
  // Check if build metadata is present
  else if (buildStart != std::string::npos)
  {
    // Set the end of the numeric (major.minor.patch) portion to the
    // start of the build metadata
    numericEnd = buildStart;
    // Pre-release info can't follow the build metadata.
    this->dataPtr->build = _versionStr.substr(numericEnd + 1);
  }

  std::string numeric = _versionStr.substr(0, numericEnd);
  std::istringstream is(numeric);
  std::string token;
  for (int i = 0; std::getline(is, token, '.'); ++i)
  {
    switch (i)
    {
      default:
      case 0:
        this->dataPtr->maj = std::stoi(token);
        break;
      case 1:
        this->dataPtr->min = std::stoi(token);
        break;
      case 2:
        this->dataPtr->patch = std::stoi(token);
        break;
    };
  }
  return true;
}
