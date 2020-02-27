/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_GUI_ALIGNTOOL_HH_
#define IGNITION_GAZEBO_GUI_ALIGNTOOL_HH_

#include <memory>

#include <ignition/rendering.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>

namespace ignition
{
namespace gazebo
{
  enum class AlignStatus
  {
    HOVER = 0,
    RESET = 1,
    ALIGN = 2,
    NONE = 3
  };

  enum class AlignAxis
  {
    ALIGN_X = 0,
    ALIGN_Y = 1,
    ALIGN_Z = 2
  };

  enum class AlignConfig
  {
    ALIGN_MIN,
    ALIGN_CENTER,
    ALIGN_MAX
  };

  class AlignToolPrivate;

  /// \brief Provides buttons for the align tool
  ///
  /// ## Configuration
  /// \<service\> : Set the service to receive align tool requests.
  class AlignTool :
    public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: AlignTool();

    /// \brief Destructor
    public: ~AlignTool() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    public slots: void OnAlignAxis(const QString &_mode);

    public slots: void OnAlignTarget(const QString &_target);

    public slots: void OnReverse(bool _reverse);

    public slots: void OnAlignConfig(const QString &_config);

    public slots: void AddStatus(const QString &_status);
    
    public: void AddStatus(const AlignStatus &_status);

    public: void MakeSolid(const rendering::NodePtr &_node);

    public: void MakeTransparent(const rendering::NodePtr &_node);

    public: rendering::VisualPtr TopLevelVisual(rendering::ScenePtr &_scene,
            rendering::VisualPtr &_visual) const;
    
    public: rendering::NodePtr TopLevelNode(rendering::ScenePtr &_scene,
            rendering::NodePtr &_node) const;

    public slots: void OnHoveredEntered();
    
    public slots: void OnHoveredExited();

    public: void Align();

    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<AlignToolPrivate> dataPtr;
  };
}
}

#endif
