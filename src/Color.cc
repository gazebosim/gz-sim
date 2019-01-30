/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <cmath>
#include <algorithm>

#include "ignition/math/Color.hh"

using namespace ignition;
using namespace math;

const Color Color::White = Color(1, 1, 1, 1);
const Color Color::Black = Color(0, 0, 0, 1);
const Color Color::Red = Color(1, 0, 0, 1);
const Color Color::Green = Color(0, 1, 0, 1);
const Color Color::Blue = Color(0, 0, 1, 1);
const Color Color::Yellow = Color(1, 1, 0, 1);
const Color Color::Magenta = Color(1, 0, 1, 1);
const Color Color::Cyan = Color(0, 1, 1, 1);

//////////////////////////////////////////////////
Color::Color()
{
}

//////////////////////////////////////////////////
Color::Color(const float _r, const float _g, const float _b, const float _a)
: r(_r), g(_g), b(_b), a(_a)
{
  this->Clamp();
}

//////////////////////////////////////////////////
Color::Color(const Color &_pt)
: r(_pt.r), g(_pt.g), b(_pt.b), a(_pt.a)
{
  this->Clamp();
}

//////////////////////////////////////////////////
Color::~Color()
{
}

//////////////////////////////////////////////////
void Color::Reset()
{
  this->r = this->g = this->b = 0;
  this->a = 1;
}

//////////////////////////////////////////////////
void Color::Set(const float _r, const float _g, const float _b, const float _a)
{
  this->r = _r;
  this->g = _g;
  this->b = _b;
  this->a = _a;

  this->Clamp();
}

//////////////////////////////////////////////////
void Color::SetFromHSV(const float _h, const float _s, const float _v)
{
  int i;
  float f, p , q, t;

  float h = static_cast<float>(static_cast<int>(_h < 0 ? 0 : _h) % 360);

  if (equal(_s, 0.0f))
  {
    // acromatic (grey)
    this->r = this->g = this->b = _v;
    return;
  }

  // sector 0 - 5
  h /= 60;

  i = static_cast<int>(floor(h));

  f = h - i;

  p = _v * (1-_s);
  q = _v * (1 - _s * f);
  t = _v * (1 - _s * (1-f));

  switch (i)
  {
    case 0:
      this->r = _v;
      this->g = t;
      this->b = p;
      break;
    case 1:
      this->r = q;
      this->g = _v;
      this->b = p;
      break;
    case 2:
      this->r = p;
      this->g = _v;
      this->b = t;
      break;
    case 3:
      this->r = p;
      this->g = q;
      this->b = _v;
      break;
    case 4:
      this->r = t;
      this->g = p;
      this->b = _v;
      break;
    case 5:
    default:
      this->r = _v;
      this->g = p;
      this->b = q;
      break;
  }

  this->Clamp();
}

//////////////////////////////////////////////////
Vector3f Color::HSV() const
{
  Vector3f hsv;

  float min = std::min(this->r, std::min(this->g, this->b));
  float max = std::max(this->r, std::max(this->g, this->b));
  float delta = max - min;

  hsv.Y() = delta / max;
  hsv.Z() = max;

  if (equal(delta, 0.0f))
  {
    hsv.X() = -1;
    hsv.Y() = 0.0;
  }
  else if (equal(this->r, min))
    hsv.X() = 3 - ((this->g - this->b) / delta);
  else if (equal(this->g, min))
    hsv.X() = 5 - ((this->b - this->r) / delta);
  else
    hsv.X() = 1 - ((this->r - this->g) / delta);

  return hsv;
}

//////////////////////////////////////////////////
Vector3f Color::YUV() const
{
  Vector3f yuv;

  yuv.X() = 0.299f*this->r + 0.587f*this->g + 0.114f*this->b;
  yuv.Y() = -0.1679f*this->r - 0.332f*this->g + 0.5f*this->b + 0.5f;
  yuv.Z() = 0.5f*this->r - 0.4189f*this->g - 0.08105f*this->b + 0.5f;

  yuv.X() = yuv.X() < 0 ? 0: yuv.X();
  yuv.X() = yuv.X() > 255 ? 255.0f: yuv.X();

  yuv.Y() = yuv.Y() < 0 ? 0: yuv.Y();
  yuv.Y() = yuv.Y() > 255 ? 255.0f: yuv.Y();

  yuv.Z() = yuv.Z() < 0 ? 0: yuv.Z();
  yuv.Z() = yuv.Z() > 255 ? 255.0f: yuv.Z();

  return yuv;
}

//////////////////////////////////////////////////
void Color::SetFromYUV(const float _y, const float _u, const float _v)
{
  this->r = _y + 1.140f*_v;
  this->g = _y - 0.395f*_u - 0.581f*_v;
  this->b = _y + 2.032f*_u;
  this->Clamp();
}

//////////////////////////////////////////////////
float Color::operator[](const unsigned int index)
{
  switch (index)
  {
    case 0:
      return this->r;
    case 1:
      return this->g;
    case 2:
      return this->b;
    case 3:
      return this->a;
    default:
      break;
  }

  return NAN_F;
}

//////////////////////////////////////////////////
Color::RGBA Color::AsRGBA() const
{
  uint8_t val8;
  unsigned int val32;

  // Convert to 32bit pattern
  // (RGBA = 8888)

  val8 = static_cast<uint8_t>(this->r * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->g * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->b * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->a * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
Color::BGRA Color::AsBGRA() const
{
  uint8_t val8;
  unsigned int val32 = 0;

  // Convert to 32bit pattern
  // (BGRA = 8888)

  val8 = static_cast<uint8_t>(this->b * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->g * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->r * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->a * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
Color::ARGB Color::AsARGB() const
{
  uint8_t val8;
  unsigned int val32 = 0;

  // Convert to 32bit pattern
  // (ARGB = 8888)

  val8 = static_cast<uint8_t>(this->a * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->r * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->g * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->b * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
Color::ABGR Color::AsABGR() const
{
  uint8_t val8;
  unsigned int val32 = 0;

  // Convert to 32bit pattern
  // (ABGR = 8888)

  val8 = static_cast<uint8_t>(this->a * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->b * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->g * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->r * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
void Color::SetFromRGBA(const Color::RGBA _v)
{
  unsigned int val32 = _v;

  // Convert from 32bit pattern
  // (RGBA = 8888)

  this->r = ((val32 >> 24) & 0xFF) / 255.0f;
  this->g = ((val32 >> 16) & 0xFF) / 255.0f;
  this->b = ((val32 >> 8) & 0xFF) / 255.0f;
  this->a = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
void Color::SetFromBGRA(const Color::BGRA _v)
{
  unsigned int val32 = _v;

  // Convert from 32bit pattern
  // (BGRA = 8888)

  this->b = ((val32 >> 24) & 0xFF) / 255.0f;
  this->g = ((val32 >> 16) & 0xFF) / 255.0f;
  this->r = ((val32 >> 8) & 0xFF) / 255.0f;
  this->a = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
void Color::SetFromARGB(const Color::ARGB _v)
{
  unsigned int val32 = _v;

  // Convert from 32bit pattern
  // (ARGB = 8888)

  this->a = ((val32 >> 24) & 0xFF) / 255.0f;
  this->r = ((val32 >> 16) & 0xFF) / 255.0f;
  this->g = ((val32 >> 8) & 0xFF) / 255.0f;
  this->b = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
void Color::SetFromABGR(const Color::ABGR _v)
{
  unsigned int val32 = _v;

  // Convert from 32bit pattern
  // (ABGR = 8888)

  this->a = ((val32 >> 24) & 0xFF) / 255.0f;
  this->b = ((val32 >> 16) & 0xFF) / 255.0f;
  this->g = ((val32 >> 8) & 0xFF) / 255.0f;
  this->r = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
Color &Color::operator=(const Color &_clr)
{
  this->r = _clr.r;
  this->g = _clr.g;
  this->b = _clr.b;
  this->a = _clr.a;

  return *this;
}

//////////////////////////////////////////////////
Color Color::operator+(const Color &_pt) const
{
  return Color(this->r + _pt.r, this->g + _pt.g,
      this->b + _pt.b, this->a + _pt.a);
}

//////////////////////////////////////////////////
Color Color::operator+(const float &_v) const
{
  return Color(this->r + _v, this->g + _v, this->b + _v, this->a + _v);
}

//////////////////////////////////////////////////
const Color &Color::operator+=(const Color &_pt)
{
  this->r += _pt.r;
  this->g += _pt.g;
  this->b += _pt.b;
  this->a += _pt.a;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
Color Color::operator-(const Color &_pt) const
{
  return Color(this->r - _pt.r, this->g - _pt.g,
      this->b - _pt.b, this->a - _pt.a);
}

//////////////////////////////////////////////////
Color Color::operator-(const float &_v) const
{
  return Color(this->r - _v, this->g - _v, this->b - _v, this->a - _v);
}

//////////////////////////////////////////////////
const Color &Color::operator-=(const Color &_pt)
{
  this->r -= _pt.r;
  this->g -= _pt.g;
  this->b -= _pt.b;
  this->a -= _pt.a;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
const Color Color::operator/(const float &_i) const
{
  return Color(this->r / _i, this->g / _i, this->b / _i, this->a / _i);
}

//////////////////////////////////////////////////
const Color Color::operator/(const Color &_pt) const
{
  return Color(this->r / _pt.r, this->g / _pt.g,
      this->b / _pt.b, this->a / _pt.a);
}

//////////////////////////////////////////////////
const Color &Color::operator/=(const Color &_pt)
{
  this->r /= _pt.r;
  this->g /= _pt.g;
  this->b /= _pt.b;
  this->a /= _pt.a;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
const Color Color::operator*(const float &_i) const
{
  return Color(this->r * _i, this->g * _i, this->b * _i, this->a * _i);
}

//////////////////////////////////////////////////
const Color Color::operator*(const Color &_pt) const
{
  return Color(this->r * _pt.r, this->g * _pt.g,
      this->b * _pt.b, this->a * _pt.a);
}

//////////////////////////////////////////////////
const Color &Color::operator*=(const Color &_pt)
{
  this->r *= _pt.r;
  this->g *= _pt.g;
  this->b *= _pt.b;
  this->a *= _pt.a;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
bool Color::operator==(const Color &_pt) const
{
  return equal(this->r, _pt.r) && equal(this->g, _pt.g) &&
         equal(this->b, _pt.b) && equal(this->a, _pt.a);
}

//////////////////////////////////////////////////
bool Color::operator!=(const Color &_pt) const
{
  return !(*this == _pt);
}

//////////////////////////////////////////////////
void Color::Clamp()
{
  this->r = this->r < 0 || isnan(this->r) ? 0: this->r;
  this->r = this->r > 1 ? this->r/255.0f: this->r;

  this->g = this->g < 0 || isnan(this->g) ? 0: this->g;
  this->g = this->g > 1 ? this->g/255.0f: this->g;

  this->b = this->b < 0 || isnan(this->b) ? 0: this->b;
  this->b = this->b > 1 ? this->b/255.0f: this->b;

  this->a = this->a < 0 || isnan(this->a) ? 0: this->a;
  this->a = this->a > 1 ? 1.0f: this->a;
}

//////////////////////////////////////////////////
float Color::R() const
{
  return this->r;
}

//////////////////////////////////////////////////
float Color::G() const
{
  return this->g;
}

//////////////////////////////////////////////////
float Color::B() const
{
  return this->b;
}

//////////////////////////////////////////////////
float Color::A() const
{
  return this->a;
}

//////////////////////////////////////////////////
float &Color::R()
{
  return this->r;
}

//////////////////////////////////////////////////
float &Color::G()
{
  return this->g;
}

//////////////////////////////////////////////////
float &Color::B()
{
  return this->b;
}

//////////////////////////////////////////////////
float &Color::A()
{
  return this->a;
}

//////////////////////////////////////////////////
void Color::R(const float _r)
{
  this->r = _r;
}

//////////////////////////////////////////////////
void Color::G(const float _g)
{
  this->g = _g;
}

//////////////////////////////////////////////////
void Color::B(const float _b)
{
  this->b = _b;
}

//////////////////////////////////////////////////
void Color::A(const float _a)
{
  this->a = _a;
}
