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

#define _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS

#include "CiVctCascadePrivate.hh"

#include "GlobalIlluminationCiVct.hh"

#include "gz/rendering/GlobalIlluminationCiVct.hh"

using namespace gz;
using namespace sim;

CiVctCascadePrivate::CiVctCascadePrivate(std::mutex &_serviceMutex,
                                         GlobalIlluminationCiVct &_creator,
                                         rendering::CiVctCascadePtr _cascade) :
  cascade(_cascade),
  creator(_creator),
  serviceMutex(_serviceMutex)
{
}

/////////////////////////////////////////////////
CiVctCascadePrivate::~CiVctCascadePrivate()
{
}

void CiVctCascadePrivate::UpdateResolution(int _axis, uint32_t _res)
{
  if (!this->creator.CascadesEditable())
    return;

  std::lock_guard<std::mutex> lock(this->serviceMutex);
  uint32_t resolution[3];
  memcpy(resolution, this->cascade->Resolution(), sizeof(resolution));
  resolution[_axis] = _res;
  this->cascade->SetResolution(resolution);
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::UpdateOctantCount(int _axis, uint32_t _count)
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  uint32_t octantCount[3];
  memcpy(octantCount, this->cascade->OctantCount(), sizeof(octantCount));
  octantCount[_axis] = _count;
  this->cascade->SetOctantCount(octantCount);
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::UpdateAreaHalfSize(int _axis, float _halfSize)
{
  if (!this->creator.CascadesEditable())
    return;

  std::lock_guard<std::mutex> lock(this->serviceMutex);
  math::Vector3d areaHalfSize = this->cascade->AreaHalfSize();
  areaHalfSize[(size_t)_axis] = static_cast<double>(_halfSize);
  this->cascade->SetAreaHalfSize(areaHalfSize);
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetAreaHalfSizeX(const float _v)
{
  this->UpdateAreaHalfSize(0, _v);
}

/////////////////////////////////////////////////
float CiVctCascadePrivate::AreaHalfSizeX() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return static_cast<float>(this->cascade->AreaHalfSize().X());
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetAreaHalfSizeY(const float _v)
{
  this->UpdateAreaHalfSize(1, _v);
}

/////////////////////////////////////////////////
float CiVctCascadePrivate::AreaHalfSizeY() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return static_cast<float>(this->cascade->AreaHalfSize().Y());
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetAreaHalfSizeZ(const float _v)
{
  this->UpdateAreaHalfSize(2, _v);
}

/////////////////////////////////////////////////
float CiVctCascadePrivate::AreaHalfSizeZ() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return static_cast<float>(this->cascade->AreaHalfSize().Z());
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetResolutionX(const uint32_t _res)
{
  this->UpdateResolution(0, _res);
}

/////////////////////////////////////////////////
uint32_t CiVctCascadePrivate::ResolutionX() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return this->cascade->Resolution()[0];
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetResolutionY(const uint32_t _res)
{
  this->UpdateResolution(1, _res);
}

/////////////////////////////////////////////////
uint32_t CiVctCascadePrivate::ResolutionY() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return this->cascade->Resolution()[1];
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetResolutionZ(const uint32_t _res)
{
  this->UpdateResolution(2, _res);
}

/////////////////////////////////////////////////
uint32_t CiVctCascadePrivate::ResolutionZ() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return this->cascade->Resolution()[2];
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetOctantCountX(const uint32_t _octantCount)
{
  this->UpdateOctantCount(0, _octantCount);
}

/////////////////////////////////////////////////
uint32_t CiVctCascadePrivate::OctantCountX() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return this->cascade->OctantCount()[0];
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetOctantCountY(const uint32_t _octantCount)
{
  this->UpdateOctantCount(1, _octantCount);
}

/////////////////////////////////////////////////
uint32_t CiVctCascadePrivate::OctantCountY() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return this->cascade->OctantCount()[1];
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetOctantCountZ(const uint32_t _octantCount)
{
  this->UpdateOctantCount(2, _octantCount);
}

/////////////////////////////////////////////////
uint32_t CiVctCascadePrivate::OctantCountZ() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return this->cascade->OctantCount()[2];
}

/////////////////////////////////////////////////
void CiVctCascadePrivate::SetThinWallCounter(const float _thinWallCounter)
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  this->cascade->SetThinWallCounter(_thinWallCounter);
}

/////////////////////////////////////////////////
float CiVctCascadePrivate::ThinWallCounter() const
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  return this->cascade->ThinWallCounter();
}
