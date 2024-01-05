/*
 * Copyright (C) 2019  Rhys Mainwaring
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

#ifndef MARITIME_WAVEFIELD_HH_
#define MARITIME_WAVEFIELD_HH_

#include <gz/msgs/param.pb.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/config.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace maritime
{
  /// \brief A class to generate a wave field.
  /// This is a port from https://github.com/srmainwaring/asv_wave_sim
  /// with some modifications.
  ///
  /// It uses SDF to parametrize the waves.
  ///
  /// ## Modes of operation: The class is designed to operate in two modes:
  ///
  /// 1. Primary: The plugin reads all the wavefield parameters from SDF.
  /// 2. Secondary: The plugin subscribes to a topic and receives the wavefield
  ///               parameters via a message.
  ///
  /// Note that the top level plugins using this class are responsible for
  /// publishing or subscribing to the wavefield topic. When configured in
  /// primary mode, you should use from the top level plugin:
  ///
  ///    Load(const std::shared_ptr<const sdf::Element> &_sdf)
  ///
  /// and:
  ///
  ///    Parameters()
  ///
  /// to read the current set of parameters and periodically send the wavefield
  /// parameters to other plugins configured in secondary mode.
  ///
  /// When configured in secondary mode, you should use from the top level
  /// plugin:
  ///
  ///    Load(const std::shared_ptr<const sdf::Element> &_sdf)
  ///
  /// and:
  ///
  ///    Load(const msgs::Param &_msg)
  ///
  /// to parse the topic and subscribe to the wavefield parameter updates.
  /// When the parameters are received, the call to the latter version of Load()
  /// will update the wavefield parameters.
  ///
  ///
  /// ## SDF parameters in primary mode:
  ///
  /// * `<topic>` (string, default: "/wavefield/parameters")
  ///    The topic for publishing wave field updates.
  ///
  /// * `<size>` (Vector2D, default: (6000 6000))
  ///   A two component vector for the size of the wave field in each direction.
  ///
  /// * `<cell_count>` (int, default: (300 300))
  ///   A two component vector for the number of grid cells in each direction.
  ///
  /// * `<number>` (int, default: 3)
  ///   The number of component waves.
  ///
  /// * `<scale>` (double, default: 1.1)
  ///   The scale between the mean and largest / smallest component waves.
  ///
  /// * `<angle>` (double, default: 0.4)
  ///   The angle between the mean wave direction and the
  ///   largest / smallest component waves.
  ///
  /// * `<steepness>` (double, default: 0.0)
  ///   A parameter in [0, 1] controlling the wave steepness with
  ///   1 being steepest.
  ///
  /// * `<amplitude>` (double, default: 0.0)
  ///   The amplitude of the mean wave in [m]. This parameter is only used when
  ///   model is CWR.
  ///
  /// * `<period>` (double, default: 5.0)
  ///   The period of the mean wave in [s].
  ///
  /// * `<phase>` (double, default: 0.0)
  ///   The phase of the mean wave.
  ///
  /// * `<direction>` (double, default: 0.0)
  ///   An angle defining mean wave direction (theta_m)
  ///
  /// * `<model>` (string, default: default)
  ///   The model used to generate component waves.
  ///   Should be either "PMS" or "CWR"
  ///
  /// * `<gain>` (double, default: 1.0)
  ///   For PMS, the multiplier applied to component amplitudes.
  ///
  /// * `<tau>` (double, default: 2.0)
  ///   Time constant used to gradually increase wavefield at startup.
  ///
  /// ## SDF parameters in secondary mode:
  ///
  /// * `<topic>` (string, default: "/wavefield/parameters")
  ///    The topic for receiving wave field updates.
  ///
  /// ## Example (primary mode)
  /// <wavefield>
  ///   <size>1000 1000</size>
  ///   <cell_count>50 50</cell_count>
  ///   <wave>
  ///     <model>PMS</model>
  ///     <period>5</period>
  ///     <number>3</number>
  ///     <scale>1.5</scale>
  ///     <gain>0.1</gain>
  ///     <direction>0.0</direction>
  ///     <angle>0.4</angle>
  ///     <tau>2.0</tau>
  ///     <amplitude>0.0</amplitude>
  ///     <steepness>0.0</steepness>
  ///   </wave>
  /// </wavefield>
  ///
  /// ## Example (secondary mode)
  /// <wavefield>
  ///   <topic>/gazebo/wavefield/parameters</topic>
  /// </wavefield>
  class Wavefield
  {
    /// \brief Constructor.
    public: Wavefield();

    /// \brief Destructor.
    public: virtual ~Wavefield();

    /// \brief Set the parameters from an SDF Element tree.
    ///
    /// \param[in] _sdf The SDF Element tree containing the wavefield parameters
    public: void Load(const std::shared_ptr<const sdf::Element> &_sdf);

    /// \brief Set the parameters from a message.
    ///
    /// \param[in] _msg The SDF Element tree containing the wavefield parameters
    public: void Load(const gz::msgs::Param &_msg);

    /// \brief Is the wavefield loaded.
    /// \return True when the wavefield has been loaded or false otherwise.
    public: bool Active() const;

    /// \brief The wavefield topic to publish/receive updates.
    /// \return The topic name.
    public: std::string Topic() const;

    /// \brief The number of wave components (3 max if visualisation required).
    public: size_t Number() const;

    /// \brief The angle between the mean wave direction and the
    ///        largest / smallest component waves.
    public: double Angle() const;

    /// \brief The scale between the mean and largest / smallest
    ///        component waves.
    public: double Scale() const;

    /// \brief A parameter in [0, 1] controlling the wave steepness
    ///        with 1 being steepest.
    public: double Steepness() const;

    /// \brief The angular frequency
    public: double AngularFrequency() const;

    /// \brief The amplitude of the mean wave in [m].
    public: double Amplitude() const;

    /// \brief The period of the mean wave in [s].
    public: double Period() const;

    /// \brief The phase of the mean wave.
    public: double Phase() const;

    /// \brief The mean wavelength.
    public: double Wavelength() const;

    /// \brief The mean wavenumber.
    public: double Wavenumber() const;

    /// \brief Time-constant for starting waves.
    public: float Tau() const;

    /// \brief Amplitude multiplier for PMS.
    public: float Gain() const;

    /// \brief A double specifiying the mean wave direction.
    public: double Direction() const;

    /// \brief Set the number of wave components (3 max).
    ///
    /// \param[in] _number The number of component waves.
    public: void SetNumber(size_t _number);

    /// \brief Set the angle parameter controlling
    /// the direction of the component waves.
    ///
    /// \param[in] _angle  The angle parameter.
    public: void SetAngle(double _angle);

    /// \brief Set the scale parameter controlling
    /// the range of amplitudes of the component waves.
    ///
    /// \param[in] _scale The scale parameter.
    public: void SetScale(double _scale);

    /// \brief Set the steepness parameter controlling
    /// the steepness of the waves. In [0, 1].
    ///
    /// \param[in] _steepness The steepness parameter.
    public: void SetSteepness(double _steepness);

    /// \brief Set the mean wave amplitude. Must be positive.
    ///
    /// \param[in] _amplitude The amplitude parameter.
    public: void SetAmplitude(double _amplitude);

    /// \brief Set the mean wave period. Must be positive.
    ///
    /// \param[in] _period The period parameter.
    public: void SetPeriod(double _period);

    /// \brief Set the mean wave phase.
    ///
    /// \param[in] _phase The phase parameter.
    public: void SetPhase(double _phase);

    /// \brief Set the time constant.
    ///
    /// \param[in] _tau The time constant.
    public: void SetTau(double _tau);

    /// \brief Set the PMS amplitude multiplier
    ///
    /// \param[in] _gain The multiplier
    public: void SetGain(double _gain);

    /// \brief Set the mean wave direction.
    ///
    /// \param[in] _direction The direction parameter, a double.
    public: void SetDirection(double _direction);

    /// \brief Access the component angular frequencies.
    public: const std::vector<double> &AngularFrequency_V() const;

    /// \brief Access the component amplitudes.
    public: const std::vector<double> &Amplitude_V() const;

    /// \brief Access the component phases.
    public: const std::vector<double> &Phase_V() const;

    /// \brief Access the steepness components.
    public: const std::vector<double> &Steepness_V() const;

    /// \brief Access the component wavenumbers.
    public: const std::vector<double> &Wavenumber_V() const;

    /// \brief Access the component directions.
    public: const std::vector<gz::math::Vector2d> &Direction_V() const;

    /// \brief Print a summary of the wave parameters to the gzmsg stream.
    public: void DebugPrint() const;

    /// A simpler version of determining wave height at a point.
    /// This method enforces that q (steepness) = 0 which allows us
    /// to caculate the wave height exactly for a given 2D point without the
    /// need to interatively solve for the position/height.
    /// \param[in] _point       The point at which we want the depth.
    /// \param[in] _time        The time at which we want the depth.
    /// \param[in] _timeInit    The time at which we want the wavefield to start
    /// \return                 The depth 'h' at the point.
    public: double ComputeDepthSimply(const gz::math::Vector3d &_point,
                                      double _time,
                                      double _timeInit = 0);

    /// \brief Compute the depth at a point directly
    /// (no sampling or interpolation).
    ///
    /// This method solves for (x, y) that when input into the
    /// Gerstner wave function
    /// gives the coordinates of the supplied parameter
    /// _point (_point.x(), _point.y()),
    /// and also computes the wave height pz at this point.
    /// The depth h = pz - point.z().
    /// This is a numerical method that uses a multi-variate
    /// Newton solver to solve
    /// the two dimensional non-linear system. In general it is not as fast as
    /// sampling from a discretised wave field with an efficient
    /// line intersection algorithm.
    ///
    /// \param[in] _point       The point at which we want the depth.
    /// \param[in] _time        The time at which we want the depth.
    /// \param[in] _timeInit    The time at which we want the wavefield to start
    /// \return                 The depth 'h' at the point.
    public: double ComputeDepthDirectly(const gz::math::Vector3d &_point,
                                        double _time,
                                        double _timeInit = 0);

    /// \brief Get all the parameters in a single message.
    /// \return A message with all the parameters.
    public: gz::msgs::Param Parameters() const;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif
