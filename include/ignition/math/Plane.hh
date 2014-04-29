/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef IGN_PLANE
#error This class should not be used directly. Use Planed.hh, \
Planef.hh, or Planei.hh.
#endif

/// \class Plane Plane.hh ignition/math.hh
/// \brief A plane and related functions.
class IGNITION_VISIBLE IGN_PLANE
{
  /// \brief Constructor
  public: IGN_PLANE();

  /// \brief Constructor from a normal and a distanec
  /// \param[in] _normal The plane normal
  /// \param[in] _offset Offset along the normal
  public: IGN_PLANE(const IGN_VECTOR3 &_normal, IGN_NUMERIC _offset = 0.0);

  /// \brief Constructor
  /// \param[in] _normal The plane normal
  /// \param[in] _size Size of the plane
  /// \param[in] _offset Offset along the normal
  public: IGN_PLANE(const IGN_VECTOR3 &_normal, const IGN_VECTOR2 &_size,
                IGN_NUMERIC _offset);

  /// \brief Destructor
  public: virtual ~IGN_PLANE();

  /// \brief Set the plane
  /// \param[in] _normal The plane normal
  /// \param[in] _size Size of the plane
  /// \param[in] _offset Offset along the normal
  public: void Set(const IGN_VECTOR3 &_normal, const IGN_VECTOR2 &_size,
                   IGN_NUMERIC offset);

  /// \brief Get distance to the plane give an origin and direction
  /// \param[in] _origin the origin
  /// \param[in] _dir a direction
  /// \return the shortest distance
  public: IGN_NUMERIC Distance(const IGN_VECTOR3 &_origin,
                          const IGN_VECTOR3 &_dir) const;

  /// \brief Get the plane size
  public: inline const IGN_VECTOR2 &GetSize() const
          { return this->size; }

  /// \brief Get the plane size
  public: inline IGN_VECTOR2 &GetSize()
          { return this->size; }

  /// \brief Get the plane offset
  public: inline const IGN_VECTOR3 &GetNormal() const
          { return this->normal; }

  /// \brief Get the plane offset
  public: inline IGN_VECTOR3 &GetNormal()
          { return this->normal; }

  /// \brief Get the plane offset
  public: inline IGN_NUMERIC GetOffset() const
          { return this->d; }

  /// \brief Equal operator
  /// \param _p another plane
  /// \return itself
  public: IGN_PLANE &operator=(const IGN_PLANE &_p);

  /// \brief Plane normal
  private: IGN_VECTOR3 normal;

  /// \brief Plane size
  private: IGN_VECTOR2 size;

  /// \brief Plane offset
  private: IGN_NUMERIC d;
};
