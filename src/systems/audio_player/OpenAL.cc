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

//#ifdef HAVE_OPENAL
#ifdef __APPLE__
# include <OpenAL/al.h>
# include <OpenAL/alc.h>
# include <OpenAL/MacOSX_OALExtensions.h>
#else
# include <AL/al.h>
# include <AL/alc.h>
# include <AL/alext.h>
#endif
//#endif

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Util.hh>
#include "OpenAL.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//#ifdef HAVE_OPENAL

/// \brief Private dat for OpenAL
class ignition::gazebo::systems::OpenAL::Implementation
{
  /// \brief OpenAL audio context pointer.
  public: ALCcontext *context;

  /// \brief OpenAL audio device pointer.
  public: ALCdevice *audioDevice;

  /// \brief OpenAL sink pointer.
  public: OpenALSinkPtr sink;
};

/// \internal
/// \brief Private data for OpenALSource
class ignition::gazebo::systems::OpenALSource::Implementation
{
  /// \brief OpenAL source index.
  public: unsigned int alSource;

  /// \brief OpenAL buffer index.
  public: unsigned int alBuffer;

  /// \brief Names of collision objects that should trigger audio
  /// playback.
  public: std::vector<std::string> collisionNames;
};


/////////////////////////////////////////////////
OpenAL::OpenAL()
: dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->context = NULL;
  this->dataPtr->audioDevice = NULL;
}

/////////////////////////////////////////////////
bool OpenAL::Load(sdf::ElementPtr _sdf)
{
  std::string deviceName = "default";

  if (_sdf && _sdf->HasElement("device"))
    deviceName = _sdf->Get<std::string>("device");

  // Open the default audio device
  if (deviceName == "default")
    this->dataPtr->audioDevice = alcOpenDevice(NULL);
  else
  {
    auto deviceList = this->DeviceList();
    if (deviceList.empty() || deviceList.find(deviceName) == deviceList.end())
      return false;
    this->dataPtr->audioDevice = alcOpenDevice(deviceName.c_str());
  }

  // Make sure that we could open the audio device
  if (this->dataPtr->audioDevice == NULL)
  {
    ignerr << "Unable to open audio device["
           << deviceName << "]\n Audio will be disabled.\n";
    return false;
  }

  // Create the audio context
  this->dataPtr->context = alcCreateContext(this->dataPtr->audioDevice, NULL);

  if (this->dataPtr->context == NULL)
  {
    ignerr << "Unable to create OpenAL Context.\nAudio will be disabled.\n";
    return false;
  }

  // Make the context current
  alcMakeContextCurrent(this->dataPtr->context);

  // Clear error code
  alGetError();

  alDistanceModel(AL_EXPONENT_DISTANCE);

  return true;
}

/////////////////////////////////////////////////
void OpenAL::Fini()
{
  if (this->dataPtr->audioDevice)
  {
    alcCloseDevice(this->dataPtr->audioDevice);
    this->dataPtr->audioDevice = NULL;
  }

  if (this->dataPtr->context)
  {
    this->dataPtr->context = alcGetCurrentContext();
    alcMakeContextCurrent(NULL);
    alcDestroyContext(this->dataPtr->context);
    this->dataPtr->context = NULL;
  }

  this->dataPtr->sink.reset();
}

/////////////////////////////////////////////////
OpenALSinkPtr OpenAL::CreateSink(sdf::ElementPtr /*_sdf*/)
{
  OpenALSinkPtr result;

  if (this->dataPtr->sink == NULL)
  {
    this->dataPtr->sink.reset(new OpenALSink);
    result  = this->dataPtr->sink;
  }
  else
    ignerr << "An OpenALSink has already been created."
           << "Only one is allowed.\n";

  return result;
}

/////////////////////////////////////////////////
OpenALSourcePtr OpenAL::CreateSource(sdf::ElementPtr _sdf)
{
  OpenALSourcePtr source;

  // Make sure the audio device has been opened
  if (!this->dataPtr->audioDevice)
  {
    ignerr << "Audio device not open\n";
    return source;
  }

  // Create a source
  source.reset(new OpenALSource());

  // Load the source
  if (!source->Load(_sdf))
  {
    ignerr << "Unable to load OpenAL source from SDF\n";
    source.reset();
  }

  // Return a pointer to the source
  return source;
}

/////////////////////////////////////////////////
std::set<std::string> OpenAL::DeviceList() const
{
  std::set<std::string> deviceList;
  const ALCchar *devices = alcGetString(NULL, ALC_DEVICE_SPECIFIER);
  while (*devices != '\0')
  {
    std::string str(devices);
    deviceList.emplace(str);
    devices += str.size() + 1;
  }
  return deviceList;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//                      OPENAL LISTENER
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
OpenALSink::OpenALSink()
{
}

/////////////////////////////////////////////////
OpenALSink::~OpenALSink()
{
}

/////////////////////////////////////////////////
bool OpenALSink::SetPose(const ignition::math::Pose3d &_pose)
{
  ALenum error;

  // Clear error state
  alGetError();

  alListener3f(AL_POSITION, _pose.Pos().X(), _pose.Pos().Y(), _pose.Pos().Z());

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set pose. Error code[" <<  error << "]\n";
    return false;
  }

  ignition::math::Matrix3d rot(_pose.Rot());

  // The first three values are the direction vector values.
  // The second three value are the up vector values.
  ALfloat orient[] = {static_cast<ALfloat>(rot(0, 0)),
                      static_cast<ALfloat>(rot(0, 1)),
                      static_cast<ALfloat>(rot(0, 2)),
                      static_cast<ALfloat>(rot(2, 0)),
                      static_cast<ALfloat>(rot(2, 1)),
                      static_cast<ALfloat>(rot(2, 2))};

  // Clear error state
  alGetError();

  alListenerfv(AL_ORIENTATION, orient);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set pose. Error code[" <<  error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSink::SetVelocity(const ignition::math::Vector3d &_vel)
{
  ALenum error;

  // Clear error state
  alGetError();

  alListener3f(AL_VELOCITY, _vel.X(), _vel.Y(), _vel.Z());
  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set velocity. Error code[" <<  error << "]\n";
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//                      OPENAL SOURCE
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
OpenALSource::OpenALSource()
: dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  // Create 1 source
  alGenSources(1, &this->dataPtr->alSource);

  // Create 1 buffer
  alGenBuffers(1, &this->dataPtr->alBuffer);
}

/////////////////////////////////////////////////
OpenALSource::~OpenALSource()
{
  alDeleteSources(1, &this->dataPtr->alSource);
  alDeleteBuffers(1, &this->dataPtr->alBuffer);
}

/////////////////////////////////////////////////
bool OpenALSource::Load(sdf::ElementPtr _sdf)
{
  if (_sdf && _sdf->HasElement("uri"))
    this->FillBufferFromFile(_sdf->Get<std::string>("uri"));
  else
  {
    ignerr << "<audio_source> is missing <uri>...</uri> element\n";
    return false;
  }

  bool result = true;

  if (_sdf->HasElement("pitch"))
    result = result && this->SetPitch(_sdf->Get<double>("pitch"));

  if (_sdf->HasElement("gain"))
    result = result && this->SetGain(_sdf->Get<double>("gain"));

  if (_sdf->HasElement("loop"))
    result = result && this->SetLoop(_sdf->Get<bool>("loop"));

  if (_sdf->HasElement("contact"))
  {
    sdf::ElementPtr collisionElem =
      _sdf->GetElement("contact")->GetElement("collision");

    while (collisionElem)
    {
      this->dataPtr->collisionNames.push_back(
          collisionElem->Get<std::string>());
      collisionElem = collisionElem->GetNextElement("collision");
    }

    result = result && !this->dataPtr->collisionNames.empty();
  }
  else
    this->Play();

  return result;
}

/////////////////////////////////////////////////
bool OpenALSource::SetPose(const ignition::math::Pose3d &_pose)
{
  ALfloat p[3] = {static_cast<float>(_pose.Pos().X()),
    static_cast<float>(_pose.Pos().Y()), static_cast<float>(_pose.Pos().Z())};
  ALenum error;

  // Clear error state
  alGetError();

  alSourcefv(this->dataPtr->alSource, AL_POSITION, p);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set position. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetVelocity(const ignition::math::Vector3d &_vel)
{
  ALenum error;
  ALfloat v[3] = {static_cast<float>(_vel.X()),
    static_cast<float>(_vel.Y()), static_cast<float>(_vel.Z())};

  // Clear error state
  alGetError();

  alSourcefv(this->dataPtr->alSource, AL_VELOCITY, v);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set velocity. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetPitch(float _pitch)
{
  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->dataPtr->alSource, AL_PITCH, _pitch);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set pitch. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetGain(float _gain)
{
  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->dataPtr->alSource, AL_GAIN, _gain);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set gain. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetLoop(bool _state)
{
  ALenum error;

  // clear error state
  alGetError();

  // Set looping state
  alSourcei(this->dataPtr->alSource, AL_LOOPING, _state);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    ignerr << " Unable to set loop. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
std::vector<std::string> OpenALSource::CollisionNames() const
{
  return this->dataPtr->collisionNames;
}

/////////////////////////////////////////////////
bool OpenALSource::OnContact() const
{
  return !this->dataPtr->collisionNames.empty();
}

/////////////////////////////////////////////////
void OpenALSource::Play()
{
  int sourceState;
  alGetSourcei(this->dataPtr->alSource, AL_SOURCE_STATE, &sourceState);

  // Play the source, if it's not already playing
  if (sourceState != AL_PLAYING)
    alSourcePlay(this->dataPtr->alSource);
}

/////////////////////////////////////////////////
void OpenALSource::Pause()
{
  int sourceState;
  alGetSourcei(this->dataPtr->alSource, AL_SOURCE_STATE, &sourceState);

  // Pause the source if it playing
  if (sourceState == AL_PLAYING)
    alSourcePause(this->dataPtr->alSource);
}

/////////////////////////////////////////////////
void OpenALSource::Stop()
{
  int sourceState;
  alGetSourcei(this->dataPtr->alSource, AL_SOURCE_STATE, &sourceState);

  // Stop the source if it is not already stopped
  if (sourceState != AL_STOPPED)
    alSourceStop(this->dataPtr->alSource);
}

/////////////////////////////////////////////////
void OpenALSource::Rewind()
{
  alSourceRewind(this->dataPtr->alSource);
}

/////////////////////////////////////////////////
bool OpenALSource::IsPlaying()
{
  int sourceState;
  alGetSourcei(this->dataPtr->alSource, AL_SOURCE_STATE, &sourceState);

  return sourceState == AL_PLAYING;
}

/////////////////////////////////////////////////
bool OpenALSource::FillBufferFromPCM(uint8_t *_pcmData,
    unsigned int _dataCount, int _sampleRate)
{
  // First detach the buffer
  alSourcei(this->dataPtr->alSource, AL_BUFFER, 0);

  // Copy raw buffer into AL buffer
  // AL_FORMAT_MONO8, AL_FORMAT_MONO16, AL_FORMAT_STEREO8,
  // AL_FORMAT_STEREO16
  alBufferData(this->dataPtr->alBuffer, AL_FORMAT_MONO16, _pcmData, _dataCount,
      _sampleRate);

  // Attach buffer to source
  alSourcei(this->dataPtr->alSource, AL_BUFFER, this->dataPtr->alBuffer);

  if (alGetError() != AL_NO_ERROR)
  {
    ignerr << "Unable to copy audio data into openAL buffer.\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void OpenALSource::FillBufferFromFile(const std::string &_audioFile)
{
  common::SystemPaths systemPaths;
  systemPaths.SetFilePathEnv(kResourcePathEnv);
  auto fullPathAudioFile = systemPaths.FindFile(_audioFile);
  //std::string fullPathAudioFile = common::find_file(_audioFile);

  // Try to open the audio file in the current directory
  FILE *testFile = fopen(fullPathAudioFile.c_str(), "r");

  // If the audio file couldn't be opened, try the gazebo paths
  if (testFile == NULL)
  {
    ignerr << "Unable to open audio file[" << _audioFile << "]\n";
  }

  uint8_t *dataBuffer = NULL;

#ifdef HAVE_FFMPEG
  unsigned int dataBufferSize;
  // Create an audio decoder
  common::AudioDecoder audioDecoder;

  // Set the audio file to decode
  audioDecoder.SetFile(fullPathAudioFile);
  audioDecoder.Decode(&dataBuffer, &dataBufferSize);

  // Copy raw buffer into AL buffer
  // AL_FORMAT_MONO8, AL_FORMAT_MONO16, AL_FORMAT_STEREO8,
  // AL_FORMAT_STEREO16
  this->FillBufferFromPCM(dataBuffer, dataBufferSize,
      audioDecoder.GetSampleRate());

#else
  std::cerr << "No FFMPEG audio decoder. Missing FFMPEG libraries.\n";
#endif

  if (dataBuffer)
    delete [] dataBuffer;

  fclose(testFile);
}

/////////////////////////////////////////////////
bool OpenALSource::HasCollisionName(const std::string &_name) const
{
  for (std::vector<std::string>::const_iterator iter =
      this->dataPtr->collisionNames.begin();
      iter != this->dataPtr->collisionNames.end(); ++iter)
  {
    if (*iter  == _name)
      return true;
  }

  return false;
}
//#endif
