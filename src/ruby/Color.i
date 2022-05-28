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

%module Color
%{
#include <sstream>
#include <gz/math/Color.hh>
#include <gz/math/config.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
%}

%include "std_string.i"

namespace gz
{
  namespace math
  {
      class Color
      {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        %rename("%(uppercase)s", %$isstatic, %$isvariable) "";
        public: static const Color White;
        public: static const Color Black;
        public: static const Color Red;
        public: static const Color Green;
        public: static const Color Blue;
        public: static const Color Yellow;
        public: static const Color Magenta;
        public: static const Color Cyan;
        public: typedef unsigned int RGBA;
        public: typedef unsigned int BGRA;
        public: typedef unsigned int ARGB;
        public: typedef unsigned int ABGR;

        public: Color();
        public: Color(const float _r, const float _g, const float _b,
                    const float _a = 1.0);
        public: Color(const Color &_clr);
        public: virtual ~Color();
        public: void Reset();
        public: void Set(const float _r = 1, const float _g = 1,
                    const float _b = 1, const float _a = 1);
        public: Vector3<float> HSV() const;
        public: void SetFromHSV(const float _h, const float _s, const float _v);
        public: Vector3<float> YUV() const;
        public: void SetFromYUV(const float _y, const float _u, const float _v);
        public: RGBA AsRGBA() const;
        public: BGRA AsBGRA() const;
        public: ARGB AsARGB() const;
        public: ABGR AsABGR() const;
        public: void SetFromRGBA(const RGBA _v);
        public: void SetFromBGRA(const BGRA _v);
        public: void SetFromARGB(const ARGB _v);
        public: void SetFromABGR(const ABGR _v);
        public: Color operator+(const Color &_pt) const;
        public: Color operator+(const float &_v) const;
        public: const Color &operator+=(const Color &_pt);
        public: Color operator-(const Color &_pt) const;
        public: Color operator-(const float &_v) const;
        public: const Color &operator-=(const Color &_pt);
        public: const Color operator/(const Color &_pt) const;
        public: const Color operator/(const float &_v) const;
        public: const Color &operator/=(const Color &_pt);
        public: const Color operator*(const Color &_pt) const;
        public: const Color operator*(const float &_v) const;
        public: const Color &operator*=(const Color &_pt);
        public: bool operator==(const Color &_pt) const;
        public: bool operator!=(const Color &_pt) const;
        private: void Clamp();
        public: float R() const;
        public: float G() const;
        public: float B() const;
        public: float A() const;
        public: void R(const float _r);
        public: void G(const float _g);
        public: void B(const float _b);
        public: void A(const float _a);
      };

      %extend Color{
        float __getitem__(const unsigned int i)
        {
          return (*$self)[i];
        }
      }

      %extend Color {
        std::string __str__() const {
          std::ostringstream out;
          out << *$self;
          return out.str();
        }
      }
  }
}
