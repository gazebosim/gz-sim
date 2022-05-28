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

%module semanticversion
%{
  #include <sstream>
	#include <gz/math/SemanticVersion.hh>
%}

%include "std_string.i"

namespace gz
{
  namespace math
  {
    class SemanticVersion
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: SemanticVersion();
      public: explicit SemanticVersion(const std::string &_v);
      public: SemanticVersion(const SemanticVersion &_copy);
      public: SemanticVersion(const unsigned int _major,
                              const unsigned int _minor = 0,
                              const unsigned int _patch = 0,
                              const std::string &_prerelease = "",
                              const std::string &_build = "");
      public: ~SemanticVersion();
      public: bool Parse(const std::string &_versionStr);
      public: std::string Version() const;
      public: unsigned int Major() const;
      public: unsigned int Minor() const;
      public: unsigned int Patch() const;
      public: std::string Prerelease() const;
      public: std::string Build() const;
      public: bool operator<(const SemanticVersion &_other) const;
      public: bool operator<=(const SemanticVersion &_other) const;
      public: bool operator>(const SemanticVersion &_other) const;
      public: bool operator>=(const SemanticVersion &_other) const;
      public: bool operator==(const SemanticVersion &_other) const;
      public: bool operator!=(const SemanticVersion &_other) const;
    };

    %extend SemanticVersion {
        std::string __str__() const {
          std::ostringstream out;
          out << *$self;
          return out.str();
        }
      }
  }
}
