/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Magnetometer.hh"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <gz/common/Profiler.hh>

#include <gz/transport/Node.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/MagnetometerSensor.hh>

#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/Magnetometer.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/** set this always to the sampling in degrees for the table below */
static constexpr const float SAMPLING_RES     =  10.0f;
static constexpr const float SAMPLING_MIN_LAT = -60.0f;
static constexpr const float SAMPLING_MAX_LAT =  60.0f;
static constexpr const float SAMPLING_MIN_LON = -180.0f;
static constexpr const float SAMPLING_MAX_LON = 180.0f;

/// Returns scalar value constrained by (min_val, max_val)
template<typename Scalar>
static inline constexpr const Scalar &constrain(
  const Scalar &val, const Scalar &min_val, const Scalar &max_val)
{
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

// declination data in degrees
static constexpr const int8_t declination_table[13][37] = \
{
  { 47, 46, 45, 43, 42, 41, 39, 37, 33, 29, 23, 16, 10, 4, -1, -6, -10, -15, -20, -27, -34, -42, -49, -56, -62, -67, -72, -74, -75, -73, -61, -22, 26, 42, 47, 48, 47 },  // NOLINT
  { 31, 31, 31, 30, 30, 30, 30, 29, 27, 24, 18, 11, 3, -4, -9, -13, -15, -18, -21, -27, -33, -40, -47, -52, -56, -57, -56, -52, -44, -30, -14, 2, 14, 22, 27, 30, 31 },  // NOLINT
  { 22, 23, 23, 23, 22, 22, 22, 23, 22, 19, 13, 5, -4, -12, -17, -20, -22, -22, -23, -25, -30, -36, -41, -45, -46, -44, -39, -31, -21, -11, -3, 4, 10, 15, 19, 21, 22 },  // NOLINT
  { 17, 17, 17, 18, 17, 17, 17, 17, 16, 13, 8, -1, -10, -18, -22, -25, -26, -25, -22, -20, -21, -25, -29, -32, -31, -28, -23, -16, -9, -3, 0, 4, 7, 11, 14, 16, 17 },  // NOLINT
  { 13, 13, 14, 14, 14, 13, 13, 12, 11, 9, 3, -5, -14, -20, -24, -25, -24, -21, -17, -12, -9, -11, -14, -17, -18, -16, -12, -8, -3, -0, 1, 3, 6, 8, 11, 12, 13 },  // NOLINT
  { 11, 11, 11, 11, 11, 10, 10, 10, 9, 6, -0, -8, -15, -21, -23, -22, -19, -15, -10, -5, -2, -2, -4, -7, -9, -8, -7, -4, -1, 1, 1, 2, 4, 7, 9, 10, 11 },  // NOLINT
  { 10, 9, 9, 9, 9, 9, 9, 8, 7, 3, -3, -10, -16, -20, -20, -18, -14, -9, -5, -2, 1, 2, 0, -2, -4, -4, -3, -2, -0, 0, 0, 1, 3, 5, 7, 9, 10 },  // NOLINT
  { 9, 9, 9, 9, 9, 9, 9, 8, 6, 1, -4, -11, -16, -18, -17, -14, -10, -5, -2, -0, 2, 3, 2, 0, -1, -2, -2, -1, -0, -1, -1, -1, 1, 3, 6, 8, 9 },  // NOLINT
  { 8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -6, -12, -15, -16, -15, -11, -7, -4, -1, 1, 3, 4, 3, 2, 1, 0, -0, -0, -1, -2, -3, -4, -2, 0, 3, 6, 8 },  // NOLINT
  { 7, 9, 10, 11, 12, 12, 12, 9, 5, -1, -7, -13, -15, -15, -13, -10, -6, -3, 0, 2, 3, 4, 4, 4, 3, 2, 1, 0, -1, -3, -5, -6, -6, -3, 0, 4, 7 },  // NOLINT
  { 5, 8, 11, 13, 14, 15, 14, 11, 5, -2, -9, -15, -17, -16, -13, -10, -6, -3, 0, 3, 4, 5, 6, 6, 6, 5, 4, 2, -1, -5, -8, -9, -9, -6, -3, 1, 5 },  // NOLINT
  { 3, 8, 11, 15, 17, 17, 16, 12, 5, -4, -12, -18, -19, -18, -16, -12, -8, -4, -0, 3, 5, 7, 9, 10, 10, 9, 7, 4, -1, -6, -10, -12, -12, -9, -5, -1, 3 },  // NOLINT
  { 3, 8, 12, 16, 19, 20, 18, 13, 4, -8, -18, -24, -25, -23, -20, -16, -11, -6, -1, 3, 7, 11, 14, 16, 17, 17, 14, 8, -0, -8, -13, -15, -14, -11, -7, -2, 3 },  // NOLINT
};

// inclination data in degrees
static constexpr const int8_t inclination_table[13][37] = \
{
  { -78, -76, -74, -72, -70, -68, -65, -63, -60, -57, -55, -54, -54, -55, -56, -57, -58, -59, -59, -59, -59, -60, -61, -63, -66, -69, -73, -76, -79, -83, -86, -87, -86, -84, -82, -80, -78 },  // NOLINT
  { -72, -70, -68, -66, -64, -62, -60, -57, -54, -51, -49, -48, -49, -51, -55, -58, -60, -61, -61, -61, -60, -60, -61, -63, -66, -69, -72, -76, -78, -80, -81, -80, -79, -77, -76, -74, -72 },  // NOLINT
  { -64, -62, -60, -59, -57, -55, -53, -50, -47, -44, -41, -41, -43, -47, -53, -58, -62, -65, -66, -65, -63, -62, -61, -63, -65, -68, -71, -73, -74, -74, -73, -72, -71, -70, -68, -66, -64 },  // NOLINT
  { -55, -53, -51, -49, -46, -44, -42, -40, -37, -33, -30, -30, -34, -41, -48, -55, -60, -65, -67, -68, -66, -63, -61, -61, -62, -64, -65, -66, -66, -65, -64, -63, -62, -61, -59, -57, -55 },  // NOLINT
  { -42, -40, -37, -35, -33, -30, -28, -25, -22, -18, -15, -16, -22, -31, -40, -48, -55, -59, -62, -63, -61, -58, -55, -53, -53, -54, -55, -55, -54, -53, -51, -51, -50, -49, -47, -45, -42 },  // NOLINT
  { -25, -22, -20, -17, -15, -12, -10, -7, -3, 1, 3, 2, -5, -16, -27, -37, -44, -48, -50, -50, -48, -44, -41, -38, -38, -38, -39, -39, -38, -37, -36, -35, -35, -34, -31, -28, -25 },  // NOLINT
  { -5, -2, 1, 3, 5, 8, 10, 13, 16, 20, 21, 19, 12, 2, -10, -20, -27, -30, -30, -29, -27, -23, -19, -17, -17, -17, -18, -18, -17, -16, -16, -16, -16, -15, -12, -9, -5 },  // NOLINT
  { 15, 18, 21, 22, 24, 26, 29, 31, 34, 36, 37, 34, 28, 20, 10, 2, -3, -5, -5, -4, -2, 2, 5, 7, 8, 7, 7, 6, 7, 7, 7, 6, 5, 6, 8, 11, 15 },  // NOLINT
  { 31, 34, 36, 38, 39, 41, 43, 46, 48, 49, 49, 46, 42, 36, 29, 24, 20, 19, 20, 21, 23, 25, 28, 30, 30, 30, 29, 29, 29, 29, 28, 27, 25, 25, 26, 28, 31 },  // NOLINT
  { 43, 45, 47, 49, 51, 53, 55, 57, 58, 59, 59, 56, 53, 49, 45, 42, 40, 40, 40, 41, 43, 44, 46, 47, 47, 47, 47, 47, 47, 47, 46, 44, 42, 41, 40, 42, 43 },  // NOLINT
  { 53, 54, 56, 57, 59, 61, 64, 66, 67, 68, 67, 65, 62, 60, 57, 55, 55, 54, 55, 56, 57, 58, 59, 59, 60, 60, 60, 60, 60, 60, 59, 57, 55, 53, 52, 52, 53 },  // NOLINT
  { 62, 63, 64, 65, 67, 69, 71, 73, 75, 75, 74, 73, 70, 68, 67, 66, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 70, 69, 67, 65, 63, 62, 62, 62 },  // NOLINT
  { 71, 71, 72, 73, 75, 77, 78, 80, 81, 81, 80, 79, 77, 76, 74, 73, 73, 73, 73, 73, 73, 74, 74, 75, 76, 77, 78, 78, 78, 78, 77, 75, 73, 72, 71, 71, 71 },  // NOLINT
};

// strength data in centi gauss
static constexpr const int8_t strength_table[13][37] = \
{
  { 62, 60, 58, 56, 54, 52, 49, 46, 43, 41, 38, 36, 34, 32, 31, 31, 30, 30, 30, 31, 33, 35, 38, 42, 46, 51, 55, 59, 62, 64, 66, 67, 67, 66, 65, 64, 62 },  // NOLINT
  { 59, 56, 54, 52, 50, 47, 44, 41, 38, 35, 32, 29, 28, 27, 26, 26, 26, 25, 25, 26, 28, 30, 34, 39, 44, 49, 54, 58, 61, 64, 65, 66, 65, 64, 63, 61, 59 },  // NOLINT
  { 54, 52, 49, 47, 45, 42, 40, 37, 34, 30, 27, 25, 24, 24, 24, 24, 24, 24, 24, 24, 25, 28, 32, 37, 42, 48, 52, 56, 59, 61, 62, 62, 62, 60, 59, 56, 54 },  // NOLINT
  { 49, 47, 44, 42, 40, 37, 35, 33, 30, 28, 25, 23, 22, 23, 23, 24, 25, 25, 26, 26, 26, 28, 31, 36, 41, 46, 51, 54, 56, 57, 57, 57, 56, 55, 53, 51, 49 },  // NOLINT
  { 43, 41, 39, 37, 35, 33, 32, 30, 28, 26, 25, 23, 23, 23, 24, 25, 26, 28, 29, 29, 29, 30, 32, 36, 40, 44, 48, 51, 52, 52, 51, 51, 50, 49, 47, 45, 43 },  // NOLINT
  { 38, 36, 35, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 24, 25, 26, 28, 30, 31, 32, 32, 32, 33, 35, 38, 42, 44, 46, 47, 46, 45, 45, 44, 43, 41, 40, 38 },  // NOLINT
  { 34, 33, 32, 32, 31, 31, 31, 30, 30, 30, 29, 28, 27, 27, 27, 28, 29, 31, 32, 33, 33, 33, 34, 35, 37, 39, 41, 42, 43, 42, 41, 40, 39, 38, 36, 35, 34 },  // NOLINT
  { 33, 33, 32, 32, 33, 33, 34, 34, 35, 35, 34, 33, 32, 31, 30, 30, 31, 32, 33, 34, 35, 35, 36, 37, 38, 40, 41, 42, 42, 41, 40, 39, 37, 36, 34, 33, 33 },  // NOLINT
  { 34, 34, 34, 35, 36, 37, 39, 40, 41, 41, 40, 39, 37, 35, 35, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 45, 45, 43, 41, 39, 37, 35, 34, 34 },  // NOLINT
  { 37, 37, 38, 39, 41, 42, 44, 46, 47, 47, 46, 45, 43, 41, 40, 39, 39, 40, 41, 41, 42, 43, 45, 46, 47, 48, 49, 50, 50, 50, 48, 46, 43, 41, 39, 38, 37 },  // NOLINT
  { 42, 42, 43, 44, 46, 48, 50, 52, 53, 53, 52, 51, 49, 47, 45, 45, 44, 44, 45, 46, 46, 47, 48, 50, 51, 53, 54, 55, 56, 55, 54, 52, 49, 46, 44, 43, 42 },  // NOLINT
  { 48, 48, 49, 50, 52, 53, 55, 56, 57, 57, 56, 55, 53, 51, 50, 49, 48, 48, 48, 49, 49, 50, 51, 53, 55, 56, 58, 59, 60, 60, 58, 56, 54, 52, 50, 49, 48 },  // NOLINT
  { 54, 54, 54, 55, 56, 57, 58, 58, 59, 58, 58, 57, 56, 54, 53, 52, 51, 51, 51, 51, 52, 53, 54, 55, 57, 58, 60, 61, 62, 61, 61, 59, 58, 56, 55, 54, 54 },  // NOLINT
};

/// \brief Private Magnetometer data class.
class gz::sim::systems::MagnetometerPrivate
{
  /// \brief A map of magnetometer entity to its sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::MagnetometerSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief True if the magnetic field is reported in gauss rather than tesla.
  public: bool useUnitsGauss = true;

  /// \brief True if the magnetic field earth frame is NED rather than ENU.
  public: bool useEarthFrameNED = true;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _magnetometer Magnetometer component.
  /// \param[in] _worldField MagneticField component.
  /// \param[in] _parent Parent entity component.
  public: void AddMagnetometer(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::Magnetometer *_magnetometer,
    const components::MagneticField *_worldField,
    const components::ParentEntity *_parent);

  /// \brief Create magnetometer sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update magnetometer sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Remove magnetometer sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveMagnetometerEntities(const EntityComponentManager &_ecm);

  unsigned get_lookup_table_index(float &val, float min, float max)
  {
    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    // limit to table bounds - required for maxima even when table spans full
    // globe range
    // limit to (table bounds - 1) because bilinear interpolation requires
    // checking (index + 1)
    val = constrain(val, min, max - SAMPLING_RES);

    return static_cast<unsigned>((-(min) + val) / SAMPLING_RES);
  }

  float get_table_data(float lat, float lon, const int8_t table[13][37])
  {
    /*
     * If the values exceed valid ranges, return zero as default
     * as we have no way of knowing what the closest real value
     * would be.
     */
    if (lat < -90.0f || lat > 90.0f ||
        lon < -180.0f || lon > 180.0f) {
      return 0.0f;
    }

    /* round down to nearest sampling resolution */
    float min_lat = static_cast<int>(lat / SAMPLING_RES) * SAMPLING_RES;
    float min_lon = static_cast<int>(lon / SAMPLING_RES) * SAMPLING_RES;

    /* find index of nearest low sampling point */
    unsigned min_lat_index =
      get_lookup_table_index(min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
    unsigned min_lon_index =
      get_lookup_table_index(min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

    const float data_sw = table[min_lat_index][min_lon_index];
    const float data_se = table[min_lat_index][min_lon_index + 1];
    const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
    const float data_nw = table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */
    const float lat_scale = constrain(
      (lat - min_lat) / SAMPLING_RES, 0.0f, 1.0f);
    const float lon_scale = constrain(
      (lon - min_lon) / SAMPLING_RES, 0.0f, 1.0f);

    const float data_min = lon_scale * (data_se - data_sw) + data_sw;
    const float data_max = lon_scale * (data_ne - data_nw) + data_nw;

    return lat_scale * (data_max - data_min) + data_min;
  }

  // return magnetic declination in degrees
  float get_mag_declination(float lat, float lon)
  {
    return get_table_data(lat, lon, declination_table);
  }

  // return magnetic field inclination in degrees
  float get_mag_inclination(float lat, float lon)
  {
    return get_table_data(lat, lon, inclination_table);
  }

  // return magnetic field strength in centi-Gauss
  float get_mag_strength(float lat, float lon)
  {
    return get_table_data(lat, lon, strength_table);
  }
};

//////////////////////////////////////////////////
Magnetometer::Magnetometer() : System(), dataPtr(
    std::make_unique<MagnetometerPrivate>())
{
}

//////////////////////////////////////////////////
Magnetometer::~Magnetometer() = default;

//////////////////////////////////////////////////
void Magnetometer::Configure(const Entity &/*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &/*_ecm*/,
                           EventManager &/*_eventMgr*/)
{
    if (_sdf->HasElement("use_units_gauss"))
    {
      this->dataPtr->useUnitsGauss = _sdf->Get<bool>("use_units_gauss");
    }
    gzdbg << "Magnetometer: using param [use_units_gauss: "
          << this->dataPtr->useUnitsGauss << "]."
          << std::endl;

    if (_sdf->HasElement("use_earth_frame_ned"))
    {
      this->dataPtr->useEarthFrameNED = _sdf->Get<bool>("use_earth_frame_ned");
    }
    gzdbg << "Magnetometer: using param [use_earth_frame_ned: "
          << this->dataPtr->useEarthFrameNED << "]."
          << std::endl;
}

//////////////////////////////////////////////////
void Magnetometer::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Magnetometer::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void Magnetometer::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Magnetometer::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the MagnetometerPrivate::Update function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveMagnetometerEntities(_ecm);
}

//////////////////////////////////////////////////
void MagnetometerPrivate::AddMagnetometer(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::Magnetometer *_magnetometer,
  const components::MagneticField *_worldField,
  const components::ParentEntity *_parent)
{
  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _magnetometer->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/magnetometer";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::MagnetometerSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::MagnetometerSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  sensor->SetWorldMagneticField(_worldField->Data());

  // Get initial pose of sensor and set the reference z pos
  // The WorldPose component was just created and so it's empty
  // We'll compute the world pose manually here
  math::Pose3d p = worldPose(_entity, _ecm);
  sensor->SetWorldPose(p);

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void MagnetometerPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("MagnetometerPrivate::CreateMagnetometerEntities");
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  // Get the world magnetic field (defined in world frame)
  auto worldField = _ecm.Component<components::MagneticField>(worldEntity);
  if (nullptr == worldField)
  {
    gzerr << "World missing magnetic field." << std::endl;
    return;
  }

  if (!this->initialized)
  {
    // Create magnetometers
    _ecm.Each<components::Magnetometer, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Magnetometer *_magnetometer,
          const components::ParentEntity *_parent)->bool
        {
          this->AddMagnetometer(_ecm, _entity, _magnetometer, worldField,
              _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create magnetometers
    _ecm.EachNew<components::Magnetometer, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Magnetometer *_magnetometer,
          const components::ParentEntity *_parent)->bool
        {
          this->AddMagnetometer(_ecm, _entity, _magnetometer, worldField,
              _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void MagnetometerPrivate::Update(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("MagnetometerPrivate::Update");
  _ecm.Each<components::Magnetometer,
            components::WorldPose>(
    [&](const Entity &_entity,
        const components::Magnetometer * /*_magnetometer*/,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          // Get the magnetometer physical position
          const math::Pose3d &magnetometerWorldPose = _worldPose->Data();
          it->second->SetWorldPose(magnetometerWorldPose);

          // Position
          auto latLonEle = sphericalCoordinates(_entity, _ecm);
          if (!latLonEle)
          {
            gzwarn << "Failed to update Magnetometer sensor enity [" << _entity
                    << "]. Spherical coordinates not set." << std::endl;
            return true;
          }

          auto lat_rad = GZ_DTOR(latLonEle.value().X());
          auto lon_rad = GZ_DTOR(latLonEle.value().Y());

          // Magnetic declination and inclination (radians)
          float declination_rad =
            get_mag_declination(
              lat_rad * 180 / GZ_PI, lon_rad * 180 / GZ_PI) * GZ_PI / 180;
          float inclination_rad =
            get_mag_inclination(
              lat_rad * 180 / GZ_PI, lon_rad * 180 / GZ_PI) * GZ_PI / 180;

          // Magnetic strength in gauss (10^5 nano tesla = 10^-2 centi gauss)
          float strength_ga =
            0.01f *
            get_mag_strength(lat_rad * 180 / GZ_PI, lon_rad * 180 / GZ_PI);

          // Magnetic intensity measured in telsa
          float strength_tesla = 1.0E-4 * strength_ga;

          // Magnetic field components are calculated in world NED frame using:
          // http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
          float H = cosf(inclination_rad);
          H *= this->useUnitsGauss ? strength_ga : strength_tesla;
          float Z_ned = tanf(inclination_rad) * H;
          float X_ned = H * cosf(declination_rad);
          float Y_ned = H * sinf(declination_rad);

          float X = X_ned;
          float Y = Y_ned;
          float Z = Z_ned;
          if (!this->useEarthFrameNED)
          {
            // Use ENU convention for earth frame.
            X = Y_ned;
            Y = X_ned;
            Z = -1.0 * Z_ned;
          }

          math::Vector3d magnetic_field_I(X, Y, Z);
          it->second->SetWorldMagneticField(magnetic_field_I);
        }
        else
        {
          gzerr << "Failed to update magnetometer: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void MagnetometerPrivate::RemoveMagnetometerEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("MagnetometerPrivate::RemoveMagnetometerEntities");
  _ecm.EachRemoved<components::Magnetometer>(
    [&](const Entity &_entity,
        const components::Magnetometer *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing magnetometer sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(Magnetometer, System,
  Magnetometer::ISystemConfigure,
  Magnetometer::ISystemPreUpdate,
  Magnetometer::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Magnetometer,
                          "gz::sim::systems::Magnetometer")
