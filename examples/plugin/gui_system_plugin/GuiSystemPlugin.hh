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

#ifndef GZ_SIM_GUISYSTEMPLUGIN_HH_
#define GZ_SIM_GUISYSTEMPLUGIN_HH_

#include <gz/sim/gui/GuiSystem.hh>

/// \brief Example of a GUI plugin that has access to entities and components.
class GuiSystemPlugin : public gz::sim::GuiSystem
{
  Q_OBJECT

    /// \brief Custom property. Use this to create properties that can be read
    /// from the QML file. See the declarations below.
    Q_PROPERTY(
      QString customProperty
      READ CustomProperty
      WRITE SetCustomProperty
      NOTIFY CustomPropertyChanged
    )

  /// \brief Constructor
  public: GuiSystemPlugin();

  /// \brief Destructor
  public: ~GuiSystemPlugin() override;

  /// \brief `gz::gui::Plugin`s can overload this function to
  /// receive custom configuration from an XML file. Here, it comes from the
  /// SDF.
  ///
  /// <gui>
  ///   <plugin ...> <!-- this is the plugin element -->
  ///     ...
  ///   </plugin>
  /// </gui>
  ///
  /// \param[in] _pluginElem SDF <plugin> element. Will be null if the plugin
  /// is loaded without any XML configuration.
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief GUI systems can overload this function to receive updated simulation
  /// state. This is called whenever the server sends state updates to the GUI.
  /// \param[in] _info Simulation information such as time.
  /// \param[in] _ecm Entity component manager, which can be used to get the
  /// latest information about entities and components.
  public: void Update(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Get the custom property as a string.
  /// \return Custom property
  public: Q_INVOKABLE QString CustomProperty() const;

  /// \brief Set the custom property from a string.
  /// \param[in] _customProperty Custom property
  public: Q_INVOKABLE void SetCustomProperty(const QString &_customProperty);

  /// \brief Notify that custom property has changed
  signals: void CustomPropertyChanged();

  /// \brief Custom property
  private: QString customProperty;
};

#endif
