/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_MODEL_HH_
#define IGNITION_GAZEBO_MODEL_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN ModelPrivate;
    //
    /// \class Model Model.hh ignition/gazebo/Model.hh
    /// \brief This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    /// All the functions provided here are meant to be used with a model
    /// entity.
    ///
    /// For example, given a model's entity, to find the value of its
    /// name component, one could use the entity-component manager (`ecm`)
    /// directly as follows:
    ///
    ///     std::string name = ecm.Component<components::Name>(entity)->Data();
    ///
    /// Using this class however, the same information can be obtained with
    /// a simpler function call:
    ///
    ///    Model model(entity);
    ///    std::string name = model.Name(ecm);
    ///
    /// \todo(louise) Store the ecm instead of passing it at every API call.
    class IGNITION_GAZEBO_VISIBLE Model {
      /// \brief Constructor
      /// \param[in] _entity Model entity
      public: explicit Model(gazebo::Entity _entity = kNullEntity);

      /// \brief Copy constructor
      /// \param[in] _model Model to copy.
      public: Model(const Model &_model);

      /// \brief Move constructor
      /// \param[in] _model Model to move.
      public: Model(Model &&_model) noexcept;

      /// \brief Move assignment operator.
      /// \param[in] _model Model component to move.
      /// \return Reference to this.
      public: Model &operator=(Model &&_model) noexcept;

      /// \brief Copy assignment operator.
      /// \param[in] _model Model to copy.
      /// \return Reference to this.
      public: Model &operator=(const Model &_model);

      /// \brief Destructor
      public: virtual ~Model();

      /// \brief Get the entity which this Model is related to.
      /// \return Model entity.
      public: gazebo::Entity Entity() const;

      /// \brief Check whether this model correctly refers to an entity that
      /// has a components::Model.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if it's a valid model in the manager.
      public: bool Valid(const EntityComponentManager &_ecm) const;

      /// \brief Get the model's unscoped name.
      /// \param[in] _ecm Entity-component manager.
      /// \return Model's name.
      public: std::string Name(const EntityComponentManager &_ecm) const;

      /// \brief Get whether this model is static.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if static.
      public: bool Static(const EntityComponentManager &_ecm) const;

      /// \brief Get whether this model has self-collide enabled.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if self-colliding.
      public: bool SelfCollide(const EntityComponentManager &_ecm) const;

      /// \brief Get whether this model has wind enabled.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if wind mode is on.
      public: bool WindMode(const EntityComponentManager &_ecm) const;

      /// \brief Get the source file where this model came from. If empty,
      /// the model wasn't loaded directly from a file, probably from an SDF
      /// string.
      /// \param[in] _ecm Entity-component manager.
      /// \return Path to the source SDF file.
      public: std::string SourceFilePath(const EntityComponentManager &_ecm)
          const;

      /// \brief Get the ID of a joint entity which is an immediate child of
      /// this model.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _name Joint name.
      /// \return Joint entity.
      /// \todo(anyone) Make const
      public: gazebo::Entity JointByName(const EntityComponentManager &_ecm,
          const std::string &_name);

      /// \brief Get the ID of a link entity which is an immediate child of
      /// this model.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _name Link name.
      /// \return Link entity.
      /// \todo(anyone) Make const
      public: gazebo::Entity LinkByName(const EntityComponentManager &_ecm,
          const std::string &_name);

      /// \brief Get all joints which are immediate children of this model.
      /// \param[in] _ecm Entity-component manager.
      /// \return All joints in this model.
      public: std::vector<gazebo::Entity> Joints(
          const EntityComponentManager &_ecm) const;

      /// \brief Get all links which are immediate children of this model.
      /// \param[in] _ecm Entity-component manager.
      /// \return All links in this model.
      public: std::vector<gazebo::Entity> Links(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the number of joints which are immediate children of this
      /// model.
      /// \param[in] _ecm Entity-component manager.
      /// \return Number of joints in this model.
      public: uint64_t JointCount(const EntityComponentManager &_ecm) const;

      /// \brief Get the number of links which are immediate children of this
      /// model.
      /// \param[in] _ecm Entity-component manager.
      /// \return Number of links in this model.
      public: uint64_t LinkCount(const EntityComponentManager &_ecm) const;

      /// \brief Set a command to change the model's pose.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _pose New model pose.
      public: void SetWorldPoseCmd(EntityComponentManager &_ecm,
          const math::Pose3d &_pose);

      /// \brief Pointer to private data.
      private: std::unique_ptr<ModelPrivate> dataPtr;
    };
    }
  }
}
#endif
