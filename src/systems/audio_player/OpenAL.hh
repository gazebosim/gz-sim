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
#ifndef IGNITION_GAZEBO_SYSTEMS_OPENAL_HH_
#define IGNITION_GAZEBO_SYSTEMS_OPENAL_HH_

#include <set>
#include <string>
#include <vector>
#include <sdf/sdf.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  class OpenALSink;
  class OpenALSource;

  /// \def OpenALSinkPtr
  /// \brief std::shared_ptr to a OpenALSink class
  typedef std::shared_ptr<OpenALSink> OpenALSinkPtr;

  /// \def OpenALSourcePtr
  /// \brief std::shared_ptr to a OpenALSource class
  typedef std::shared_ptr<OpenALSource> OpenALSourcePtr;

  /// \brief ToDo.
  class OpenAL
  {
    /// \brief Constructor
    public: OpenAL();

    /// \brief Load the OpenAL server.
    /// \return True on success.
    public: bool Load(sdf::ElementPtr _sdf = sdf::ElementPtr());

    /// \brief Finalize.
    public: void Fini();

    /// \brief Create an OpenALSource object.
    /// \param[in] _sdf SDF element parameters for an audio_source.
    /// \return A pointer to an OpenALSource object.
    public: OpenALSourcePtr CreateSource(sdf::ElementPtr _sdf);

    /// \brief Create an audio listener. Currenly, only one listener may be
    /// created.
    /// \param[in] _sdf SDF element parameters for an audio_source.
    /// \return A pointer to an OpenALSink object.
    public: OpenALSinkPtr CreateSink(sdf::ElementPtr _sdf);

    /// \brief Get a list of available audio devices
    /// \return A list of audio device names
    public: std::set<std::string> DeviceList() const;

    /// \brief Private data pointer.
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };

  /// \brief OpenAL Listener. This can be thought of as a microphone.
  class OpenALSink
  {
    /// \brief Constructor
    public: OpenALSink();

    /// \brief Destructor
    public: virtual ~OpenALSink();

    /// \brief Set the position of the sink.
    /// \param[in] _pose New pose of the sink.
    /// \return True on success.
    public: bool SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Set the velocity of the sink
    /// \param[in] _vel Velocity of the sink.
    /// \return True on success.
    public: bool SetVelocity(const ignition::math::Vector3d &_vel);
  };

  /// \class OpenALSource OpenALSource.hh util/util.hh
  /// \brief OpenAL Source. This can be thought of as a speaker.
  class OpenALSource
  {
    /// \brief Constructor.
    public: OpenALSource();

    /// \brief Destructor.
    public: virtual ~OpenALSource();

    /// \brief Load the source from sdf.
    /// \param[in] _sdf SDF element parameters for an audio_source.
    /// \return True on success.
    public: bool Load(sdf::ElementPtr _sdf);

    /// \brief Set the position of the source.
    /// \param[in] _pose New pose of the source.
    /// \return True on success.
    public: bool SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Set the velocity of the source.
    /// \param[in] _vel New velocity of the source.
    /// \return True on success.
    public: bool SetVelocity(const ignition::math::Vector3d &_vel);

    /// \brief Set the pitch of the source.
    /// \param[in] _p Pitch value.
    /// \return True on success.
    public: bool SetPitch(float _p);

    /// \brief Set the pitch of the source.
    /// \param[in] _g Gain value.
    /// \return True on success.
    public: bool SetGain(float _g);

    /// \brief Set whether the source loops the audio.
    /// \param[in] _state True to cause playback to loop.
    /// \return True on success.
    public: bool SetLoop(bool _state);

    /// \brief Return true if the audio source is played on contact with
    /// another object. Contact is determine based on a set of
    /// collision objects.
    /// \return True if audio is played on contact.
    /// \sa AddCollision()
    public: bool OnContact() const;

    /// \brief Get a vector of all the collision names.
    /// \return All the collision names used to trigger audio playback on
    /// contact.
    public: std::vector<std::string> CollisionNames() const;

    /// \brief Get whether the source has a collision name set.
    /// \param[in] _name Name of a collision to check for.
    /// \return True if the collision name was found.
    public: bool HasCollisionName(const std::string &_name) const;

    /// \brief Play a sound
    public: void Play();

    /// \brief Pause a sound
    public: void Pause();

    /// \brief Stop a sound
    public: void Stop();

    /// \brief Rewind the sound to the beginning
    public: void Rewind();

    /// \brief Is the audio playing
    public: bool IsPlaying();

    /// \brief Fill the OpenAL audio buffer from PCM data
    /// \param[in] _pcmData Pointer to the PCM audio data.
    /// \param[in] _dataCount Size of the PCM data.
    /// \param[in] _sampleRate Sample rate for the PCM data.
    /// \return True on success.
    public: bool FillBufferFromPCM(uint8_t *_pcmData, unsigned int _dataCount,
                                   int _sampleRate);

    /// \brief Fill the OpenAL audio buffer with data from a sound file.
    /// \param[in] _audioFile Name and an audio file.
    public: void FillBufferFromFile(const std::string &_audioFile);

    /// \brief Private data pointer.
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}
}

#endif
