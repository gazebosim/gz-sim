/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_SIM_GUI_SPAWN_HH_
#define GZ_SIM_GUI_SPAWN_HH_

#include <memory>

#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
  class SpawnPrivate;

  /// \brief Allows to spawn models and lights using the spawn gui events or
  /// drag and drop.
  class Spawn : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Text for popup error
    Q_PROPERTY(
      QString errorPopupText
      READ ErrorPopupText
      WRITE SetErrorPopupText
      NOTIFY ErrorPopupTextChanged
    )

    /// \brief Constructor
    public: Spawn();

    /// \brief Destructor
    public: ~Spawn() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Handle drop events.
    /// \param[in] _event Event with drop information.
    public: void OnDropped(const gz::gui::events::DropOnScene *_event);

    /// \brief Get the text for the popup error message
    /// \return The error text
    public: Q_INVOKABLE QString ErrorPopupText() const;

    /// \brief Set the text for the popup error message
    /// \param[in] _errorTxt The error text
    public: Q_INVOKABLE void SetErrorPopupText(const QString &_errorTxt);

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Notify the popup error text has changed
    signals: void ErrorPopupTextChanged();

    /// \brief Notify that an error has occurred (opens popup)
    /// Note that the function name needs to start with lowercase in order for
    /// the connection to work on the QML side
    signals: void popupError();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<SpawnPrivate> dataPtr;
  };
}
}

#endif
