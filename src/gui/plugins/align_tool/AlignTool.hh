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

#ifndef GZ_SIM_GUI_ALIGNTOOL_HH_
#define GZ_SIM_GUI_ALIGNTOOL_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>
#include <gz/gui/Plugin.hh>
#include <gz/rendering/Node.hh>

namespace gz
{
namespace sim
{
  /// \brief Enumeration of the states within the Align Tool.
  enum class AlignState
  {
    /// \brief Indicates the user is currently hovering the mouse over
    /// an align button
    HOVER = 0,
    /// \brief Indicates a reset of the currently placed nodes, only occurs
    /// on a hover exit if the align button has not been clicked
    RESET = 1,
    /// \brief Indicates the user has clicked the align button
    ALIGN = 2,
    /// \brief Indicates the user is currently not utilizing the align tool
    NONE = 3
  };

  /// \brief Enumeration of the axes to be aligned relative to.
  enum class AlignAxis
  {
    /// \brief Indicates an alignment relative to the x axis
    ALIGN_X = 0,
    /// \brief Indicates an alignment relative to the y axis
    ALIGN_Y = 1,
    /// \brief Indicates an alignment relative to the z axis
    ALIGN_Z = 2
  };

  enum class AlignConfig
  {
    ALIGN_MIN,
    ALIGN_MID,
    ALIGN_MAX
  };

  class AlignToolPrivate;

  /// \brief Provides buttons for the align tool
  ///
  /// ## Configuration
  /// \<service\> : Set the service to receive align tool requests.
  class AlignTool :
    public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: AlignTool();

    /// \brief Destructor
    public: ~AlignTool() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    /// \brief Callback to update the axis type.
    /// \param[in] _mode New axis type
    public slots: void OnAlignAxis(const QString &_mode);

    /// \brief Callback to update the target type.
    /// \param[in] _target New target type
    public slots: void OnAlignTarget(const QString &_target);

    /// \brief Callback to update the axis config.
    /// \param[in] _mode New config type
    public slots: void OnAlignConfig(const QString &_mode);

    /// \brief Callback to update if the align orientation is reversed.
    /// \param[in] _reverse The reverse boolean
    public slots: void OnReverse(bool _reverse);

    /// \brief Callback to make whenever a hover state is entered on a button.
    public slots: void OnHoveredEntered();

    /// \brief Callback to make whenever a hover state is exited on a button.
    public slots: void OnHoveredExited();

    /// \brief Callback to the align state the execution queue.
    public slots: void OnAlign();

    /// \brief Callback to add a state to the execution queue.
    /// \param[in] _state New state to add by enum AlignState
    public: void AddState(const AlignState &_state);

    /// \brief Updates the node to increase its transparency or reset
    /// back to its original transparency value, an opaque call requires
    /// a previous transparent call, otherwise, no action will be taken
    /// \param[in] _node The node to update.
    /// \param[in] _makeTransparent true if updating to increase transparency,
    /// false to set back to original transparency values (make more opaque)
    public: void UpdateTransparency(const rendering::NodePtr &_node,
        bool _makeTransparent);

    /// \brief The function call to execute a state from the queue.  This
    /// function makes rendering calls and should only be executed within
    /// the render thread.
    public: void Align();

    /// \brief Returns the top level visual of the passed in visual within
    /// a given scene.
    /// \param[in] _scene The scene to check
    /// \param[in] _visual The visual to get the top level visual for
    public: rendering::VisualPtr TopLevelVisual(rendering::ScenePtr &_scene,
            rendering::VisualPtr &_visual) const;

    /// \brief Returns the top level node of the passed in node within
    /// a given scene.
    /// \param[in] _scene The scene to check
    /// \param[in] _node The node to get the top level node for
    public: rendering::NodePtr TopLevelNode(rendering::ScenePtr &_scene,
            rendering::NodePtr &_node) const;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<AlignToolPrivate> dataPtr;
  };
}
}

#endif
