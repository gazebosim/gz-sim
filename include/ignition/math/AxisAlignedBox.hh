/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_AXISALIGNEDBOX_HH_
#define IGNITION_MATH_AXISALIGNEDBOX_HH_

#include <iostream>
#include <tuple>
#include <ignition/math/config.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Line3.hh>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Material.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Forward declaration of private data
    class AxisAlignedBoxPrivate;

    /// \class AxisAlignedBox AxisAlignedBox.hh ignition/math/AxisAlignedBox.hh
    /// \brief Mathematical representation of a box that is aligned along
    /// an X,Y,Z axis.
    class IGNITION_MATH_VISIBLE AxisAlignedBox
    {
      /// \brief Default constructor. This constructor will set the box's
      /// minimum and maximum corners to the highest (max) and lowest
      /// floating point values available to indicate that it is uninitialized.
      /// The default box does not intersect any other boxes or contain any
      /// points since it has no extent. Its center is the origin and its side
      /// lengths are 0.
      public: AxisAlignedBox();

      /// \brief Constructor. This constructor will compute the box's
      /// minimum and maximum corners based on the two arguments.
      /// \param[in] _vec1 One corner of the box
      /// \param[in] _vec2 Another corner of the box
      public: AxisAlignedBox(const Vector3d &_vec1, const Vector3d &_vec2);

      /// \brief Constructor. This constructor will compute the box's
      /// minimum and maximum corners based on the arguments.
      /// \param[in] _vec1X One corner's X position
      /// \param[in] _vec1Y One corner's Y position
      /// \param[in] _vec1Z One corner's Z position
      /// \param[in] _vec2X Other corner's X position
      /// \param[in] _vec2Y Other corner's Y position
      /// \param[in] _vec2Z Other corner's Z position
      public: AxisAlignedBox(double _vec1X, double _vec1Y, double _vec1Z,
                  double _vec2X, double _vec2Y, double _vec2Z);

      /// \brief Copy Constructor
      /// \param[in]  _b AxisAlignedBox to copy
      public: AxisAlignedBox(const AxisAlignedBox &_b);

      /// \brief Destructor
      public: virtual ~AxisAlignedBox();

      /// \brief Get the length along the x dimension
      /// \return Double value of the length in the x dimension
      public: double XLength() const;

      /// \brief Get the length along the y dimension
      /// \return Double value of the length in the y dimension
      public: double YLength() const;

      /// \brief Get the length along the z dimension
      /// \return Double value of the length in the z dimension
      public: double ZLength() const;

      /// \brief Get the size of the box
      /// \return Size of the box
      public: math::Vector3d Size() const;

      /// \brief Get the box center
      /// \return The center position of the box
      public: math::Vector3d Center() const;

      /// \brief Merge a box with this box
      /// \param[in]  _box AxisAlignedBox to add to this box
      public: void Merge(const AxisAlignedBox &_box);

      /// \brief Assignment operator. Set this box to the parameter
      /// \param[in]  _b AxisAlignedBox to copy
      /// \return The new box.
      public: AxisAlignedBox &operator=(const AxisAlignedBox &_b);

      /// \brief Addition operator. result = this + _b
      /// \param[in] _b AxisAlignedBox to add
      /// \return The new box
      public: AxisAlignedBox operator+(const AxisAlignedBox &_b) const;

      /// \brief Addition set operator. this = this + _b
      /// \param[in] _b AxisAlignedBox to add
      /// \return This new box
      public: const AxisAlignedBox &operator+=(const AxisAlignedBox &_b);

      /// \brief Equality test operator
      /// \param[in] _b AxisAlignedBox to test
      /// \return True if equal
      public: bool operator==(const AxisAlignedBox &_b) const;

      /// \brief Inequality test operator
      /// \param[in] _b AxisAlignedBox to test
      /// \return True if not equal
      public: bool operator!=(const AxisAlignedBox &_b) const;

      /// \brief Subtract a vector from the min and max values
      /// \param _v The vector to use during subtraction
      /// \return The new box
      public: AxisAlignedBox operator-(const Vector3d &_v);

      /// \brief Output operator
      /// \param[in] _out Output stream
      /// \param[in] _b AxisAlignedBox to output to the stream
      /// \return The stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const ignition::math::AxisAlignedBox &_b)
      {
        _out << "Min[" << _b.Min() << "] Max[" << _b.Max() << "]";
        return _out;
      }

      /// \brief Get the minimum corner.
      /// \return The Vector3d that is the minimum corner of the box.
      public: const Vector3d &Min() const;

      /// \brief Get the maximum corner.
      /// \return The Vector3d that is the maximum corner of the box.
      public: const Vector3d &Max() const;

      /// \brief Get a mutable version of the minimum corner.
      /// \return The Vector3d that is the minimum corner of the box.
      public: Vector3d &Min();

      /// \brief Get a mutable version of the maximum corner.
      /// \return The Vector3d that is the maximum corner of the box.
      public: Vector3d &Max();

      /// \brief Test box intersection. This test will only work if
      /// both box's minimum corner is less than or equal to their
      /// maximum corner.
      /// \param[in] _box AxisAlignedBox to check for intersection with
      /// this box.
      /// \return True if this box intersects _box.
      public: bool Intersects(const AxisAlignedBox &_box) const;

      /// \brief Check if a point lies inside the box.
      /// \param[in] _p Point to check.
      /// \return True if the point is inside the box.
      public: bool Contains(const Vector3d &_p) const;

      /// \brief Check if a ray (origin, direction) intersects the box.
      /// \param[in] _origin Origin of the ray.
      /// \param[in] _dir Direction of the ray. This ray will be normalized.
      /// \param[in] _min Minimum allowed distance.
      /// \param[in] _max Maximum allowed distance.
      /// \return A boolean
      public: bool IntersectCheck(const Vector3d &_origin, const Vector3d &_dir,
                  const double _min, const double _max) const;

      /// \brief Check if a ray (origin, direction) intersects the box.
      /// \param[in] _origin Origin of the ray.
      /// \param[in] _dir Direction of the ray. This ray will be normalized.
      /// \param[in] _min Minimum allowed distance.
      /// \param[in] _max Maximum allowed distance.
      /// \return A boolean and double tuple. The boolean value is true
      /// if the line intersects the box.
      ///
      /// The double is the distance from
      /// the ray's start  to the closest intersection point on the box,
      /// minus the _min distance. For example, if _min == 0.5 and the
      /// intersection happens at a distance of 2.0 from _origin then returned
      /// distance is 1.5.
      ///
      /// The double value is zero when the boolean value is false.
      public: std::tuple<bool, double> IntersectDist(
                  const Vector3d &_origin, const Vector3d &_dir,
                  const double _min, const double _max) const;

      /// \brief Check if a ray (origin, direction) intersects the box.
      /// \param[in] _origin Origin of the ray.
      /// \param[in] _dir Direction of the ray. This ray will be normalized.
      /// \param[in] _min Minimum allowed distance.
      /// \param[in] _max Maximum allowed distance.
      /// \return A boolean, double, Vector3d tuple. The boolean value is true
      /// if the line intersects the box.
      ///
      /// The double is the distance from the ray's start to the closest
      /// intersection point on the box,
      /// minus the _min distance. For example, if _min == 0.5 and the
      /// intersection happens at a distance of 2.0 from _origin then returned
      /// distance is 1.5.
      /// The double value is zero when the boolean value is false. The
      ///
      /// Vector3d is the intersection point on the box. The Vector3d value
      /// is zero if the boolean value is false.
      public: std::tuple<bool, double, Vector3d> Intersect(
                  const Vector3d &_origin, const Vector3d &_dir,
                  const double _min, const double _max) const;

      /// \brief Check if a line intersects the box.
      /// \param[in] _line The line to check against this box.
      /// \return A boolean, double, Vector3d tuple. The boolean value is true
      /// if the line intersects the box. The double is the distance from
      /// the line's start to the closest intersection point on the box.
      /// The double value is zero when the boolean value is false. The
      /// Vector3d is the intersection point on the box. The Vector3d value
      /// is zero if the boolean value is false.
      public: std::tuple<bool, double, Vector3d> Intersect(
                  const Line3d &_line) const;

      /// \brief Get the volume of the box in m^3.
      /// \return Volume of the box in m^3.
      public: double Volume() const;

      /// \brief Compute the cylinder's density given a mass value. The
      /// cylinder is assumed to be solid with uniform density. This
      /// function requires the cylinder's radius and length to be set to
      /// values greater than zero. The Material of the cylinder is ignored.
      /// \param[in] _mass Mass of the cylinder, in kg. This value should be
      /// greater than zero.
      /// \return Density of the cylinder in kg/m^3. A negative value is
      /// returned if radius, length or _mass is <= 0.
      public: double DensityFromMass(const double _mass) const;

      /// \brief Set the density of this box based on a mass value.
      /// Density is computed using
      /// double DensityFromMass(const double _mass) const. The
      /// box is assumed to be solid with uniform density. This
      /// function requires the box's size to be set to
      /// values greater than zero. The existing Material density value is
      /// overwritten only if the return value from this true.
      /// \param[in] _mass Mass of the box, in kg. This value should be
      /// greater than zero.
      /// \return True if the density was set. False is returned if the
      /// box's size or the _mass value are <= 0.
      /// \sa double DensityFromMass(const double _mass) const
      public: bool SetDensityFromMass(const double _mass);

      /// \brief Get the material associated with this box.
      /// \return The material assigned to this box.
      public: const ignition::math::Material &Material() const;

      /// \brief Set the material associated with this box.
      /// \param[in] _mat The material assigned to this box
      public: void SetMaterial(const ignition::math::Material &_mat);

      /// \brief Get the mass matrix for this box. This function
      /// is only meaningful if the box's size and material
      /// have been set.
      /// \param[out] _massMatrix The computed mass matrix will be stored
      /// here.
      /// \return False if computation of the mass matrix failed, which
      /// could be due to an invalid size (<=0) or density (<=0).
      public: bool MassMatrix(MassMatrix3d &_massMat) const;

      /// \brief Clip a line to a dimension of the box.
      /// This is a helper function to Intersects
      /// \param[in] _d Dimension of the box(0, 1, or 2).
      /// \param[in] _line Line to clip
      /// \param[in,out] _low Close distance
      /// \param[in,out] _high Far distance
      private: bool ClipLine(const int _d, const Line3d &_line,
                   double &_low, double &_high) const;

      /// \brief Private data pointer
      private: AxisAlignedBoxPrivate *dataPtr;
    };
    }
  }
}
#endif
