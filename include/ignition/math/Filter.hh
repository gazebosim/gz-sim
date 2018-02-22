/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_FILTER_HH_
#define IGNITION_MATH_FILTER_HH_

#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Filter Filter.hh ignition/math/Filter.hh
    /// \brief Filter base class
    template <class T>
    class Filter
    {
      /// \brief Destructor.
      public: virtual ~Filter() {}

      /// \brief Set the output of the filter.
      /// \param[in] _val New value.
      public: virtual void Set(const T &_val)
      {
        y0 = _val;
      }

      /// \brief Set the cutoff frequency and sample rate.
      /// \param[in] _fc Cutoff frequency.
      /// \param[in] _fs Sample rate.
      public: virtual void Fc(double _fc, double _fs) = 0;

      /// \brief Get the output of the filter.
      /// \return Filter's output.
      public: virtual const T &Value() const
      {
        return y0;
      }

      /// \brief Output.
      protected: T y0{};
    };

    /// \class OnePole Filter.hh ignition/math/Filter.hh
    /// \brief A one-pole DSP filter.
    /// \sa http://www.earlevel.com/main/2012/12/15/a-one-pole-filter/
    template <class T>
    class OnePole : public Filter<T>
    {
      /// \brief Constructor.
      public: OnePole() = default;

      /// \brief Constructor.
      /// \param[in] _fc Cutoff frequency.
      /// \param[in] _fs Sample rate.
      public: OnePole(double _fc, double _fs)
      {
        this->Fc(_fc, _fs);
      }

      // Documentation Inherited.
      public: virtual void Fc(double _fc, double _fs)
      {
        b1 = exp(-2.0 * IGN_PI * _fc / _fs);
        a0 = 1.0 - b1;
      }

      /// \brief Update the filter's output.
      /// \paran[in] _x Input value.
      /// \return The filter's current output.
      public: const T& Process(const T &_x)
      {
        this->y0 = a0 * _x + b1 * this->y0;
        return this->y0;
      }

      /// \brief Input gain control.
      protected: double a0 = 0;

      /// \brief Gain of the feedback.
      protected: double b1 = 0;
    };

    /// \class OnePoleQuaternion Filter.hh ignition/math/Filter.hh
    /// \brief One-pole quaternion filter.
    class OnePoleQuaternion : public OnePole<math::Quaterniond>
    {
      /// \brief Constructor.
      public: OnePoleQuaternion()
      {
        this->Set(math::Quaterniond(1, 0, 0, 0));
      }

      /// \brief Constructor.
      /// \param[in] _fc Cutoff frequency.
      /// \param[in] _fs Sample rate.
      public: OnePoleQuaternion(double _fc, double _fs)
        : OnePole<math::Quaterniond>(_fc, _fs)
      {
        this->Set(math::Quaterniond(1, 0, 0, 0));
      }

      /// \brief Update the filter's output.
      /// \paran[in] _x Input value.
      /// \return The filter's current output.
      public: const math::Quaterniond& Process(
                  const math::Quaterniond &_x)
      {
        y0 = math::Quaterniond::Slerp(a0, y0, _x);
        return y0;
      }
    };

    /// \class OnePoleVector3 Filter.hh ignition/math/Filter.hh
    /// \brief One-pole vector3 filter.
    class OnePoleVector3 : public OnePole<math::Vector3d>
    {
      /// \brief Constructor.
      public: OnePoleVector3()
      {
        this->Set(math::Vector3d(0, 0, 0));
      }

      /// \brief Constructor.
      /// \param[in] _fc Cutoff frequency.
      /// \param[in] _fs Sample rate.
      public: OnePoleVector3(double _fc, double _fs)
        : OnePole<math::Vector3d>(_fc, _fs)
      {
        this->Set(math::Vector3d(0, 0, 0));
      }
    };

    /// \class BiQuad Filter.hh ignition/math/Filter.hh
    /// \brief Bi-quad filter base class.
    /// \sa http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform/
    template <class T>
    class BiQuad : public Filter<T>
    {
      /// \brief Constructor.
      public: BiQuad() = default;

      /// \brief Constructor.
      /// \param[in] _fc Cutoff frequency.
      /// \param[in] _fs Sample rate.
      public: BiQuad(double _fc, double _fs)
      {
        this->Fc(_fc, _fs);
      }

      // Documentation Inherited.
      public: void Fc(double _fc, double _fs)
      {
        this->Fc(_fc, _fs, 0.5);
      }

      /// \brief Set the cutoff frequency, sample rate and Q coefficient.
      /// \param[in] _fc Cutoff frequency.
      /// \param[in] _fs Sample rate.
      /// \param[in] _q Q coefficient.
      public: void Fc(double _fc, double _fs, double _q)
      {
        double k = tan(IGN_PI * _fc / _fs);
        double kQuadDenom = k * k + k / _q + 1.0;
        this->a0 = k * k/ kQuadDenom;
        this->a1 = 2 * this->a0;
        this->a2 = this->a0;
        this->b0 = 1.0;
        this->b1 = 2 * (k * k - 1.0) / kQuadDenom;
        this->b2 = (k * k - k / _q + 1.0) / kQuadDenom;
      }

      /// \brief Set the current filter's output.
      /// \param[in] _val New filter's output.
      public: virtual void Set(const T &_val)
      {
        this->y0 = this->y1 = this->y2 = this->x1 = this->x2 = _val;
      }

      /// \brief Update the filter's output.
      /// \param[in] _x Input value.
      /// \return The filter's current output.
      public: virtual const T& Process(const T &_x)
      {
        this->y0 = this->a0 * _x +
                   this->a1 * this->x1 +
                   this->a2 * this->x2 -
                   this->b1 * this->y1 -
                   this->b2 * this->y2;

        this->x2 = this->x1;
        this->x1 = _x;
        this->y2 = this->y1;
        this->y1 = this->y0;
        return this->y0;
      }

      /// \brief Input gain control coefficients.
      protected: double a0 = 0,
                        a1 = 0,
                        a2 = 0,
                        b0 = 0,
                        b1 = 0,
                        b2 = 0;

      /// \brief Gain of the feedback coefficients.
      protected: T x1{}, x2{}, y1{}, y2{};
    };

    /// \class BiQuadVector3 Filter.hh ignition/math/Filter.hh
    /// \brief BiQuad vector3 filter
    class BiQuadVector3 : public BiQuad<math::Vector3d>
    {
      /// \brief Constructor.
      public: BiQuadVector3()
      {
        this->Set(math::Vector3d(0, 0, 0));
      }

      /// \brief Constructor.
      /// \param[in] _fc Cutoff frequency.
      /// \param[in] _fs Sample rate.
      public: BiQuadVector3(double _fc, double _fs)
        : BiQuad<math::Vector3d>(_fc, _fs)
      {
        this->Set(math::Vector3d(0, 0, 0));
      }
    };
    }
  }
}

#endif
