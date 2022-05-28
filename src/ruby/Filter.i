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

%module filter
%{
#include <gz/math/config.hh>
#include <gz/math/Filter.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
%}

%import Quaternion.i

namespace gz
{
namespace math
{
    template <class T>
    class Filter
    {
        public: virtual ~Filter();
        public: virtual void Set(const T &_val);
        public: virtual void Fc(double _fc, double _fs) = 0;
        public: virtual const T &Value() const;
    };

    %template(Filterd) Filter<double>;
    %template(Filterq) Filter<Quaternion<double>>;

    template <class T>
    class OnePole : public Filter<T>
    {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        public: OnePole() = default;
        public: OnePole(double _fc, double _fs);
        public: virtual void Fc(double _fc, double _fs) override;
        public: const T& Process(const T &_x);
    };


    %template(OnePoled) OnePole<double>;

    class OnePoleQuaternion
    {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        public: virtual const Quaternion<double> &Value() const;
        public: OnePoleQuaternion();
        public: OnePoleQuaternion(double _fc, double _fs)
            : OnePole<Quaternion<double>>(_fc, _fs);
        public: const Quaternion<double>& Process(
            const Quaternion<double> &_x);
    };


    class OnePoleVector3
    {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        public: virtual const Vector3<double> &Value() const;
        public: const Vector3<double>& Process(const Vector3<double> &_x);
        public: OnePoleVector3();
        public: OnePoleVector3(double _fc, double _fs)
            : OnePole<Vector3<double>>(_fc, _fs);
    };

    template <class T>
    class BiQuad : public Filter<T>
    {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        public: BiQuad() = default;
        public: BiQuad(double _fc, double _fs);
        public: void Fc(double _fc, double _fs) override;
        public: void Fc(double _fc, double _fs, double _q);
        public: virtual void Set(const T &_val) override;
        public: virtual const T& Process(const T &_x);
    };

    %template(BiQuadd) BiQuad<double>;

    class BiQuadVector3
    {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        public: virtual const Vector3<double>& Process(const Vector3<double> &_x);
        public: virtual const Vector3<double> &Value() const;
        public: BiQuadVector3();
        public: BiQuadVector3(double _fc, double _fs)
            : BiQuad<Vector3<double>>(_fc, _fs);
    };
}
}
