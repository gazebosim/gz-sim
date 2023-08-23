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
#ifndef GZ_GUI_GUIRELAY_HH_
#define GZ_GUI_GUIRELAY_HH_

#include <gz/gui/Application.hh>
#include <gz/gui/Export.hh>
#include <gz/gui/MainWindow.hh>

namespace gz
{
namespace sim
{
namespace test
{
/// \brief Helper class to be used in internal tests. It allows receiving
/// events emitted in the GUI.
///
/// ## Usage
///
///  // Instantiate the class
///  test::GuiRelay guiRelay;
///
///  // Register callback, for example:
///  guiRelay.OnQEvent([&](QEvent *_event)
///    {
///      if (_event.type() == gz::gui::events::Render::kType)
///      {
///        // Do something
///      }
///    }
///
class GuiRelay : public QObject
{
  Q_OBJECT

  /// \brief Constructor
  public: GuiRelay()
  {
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->
      installEventFilter(this);
  }

  /// \brief Destructor
  public: ~GuiRelay() = default;

  /// \brief Wrapper around Qt's event filter
  /// \param[in] _cb Function to be called on every received QEvent
  public: GuiRelay &OnQEvent(std::function<void (QEvent *)> _cb)
  {
    this->forwardEvent = std::move(_cb);
    return *this;
  }

  /// \brief Documentation inherited
  public: bool eventFilter(QObject *_obj, QEvent *_event) override
  {
    if (this->forwardEvent)
      this->forwardEvent(_event);

    // Standard event processing
    return QObject::eventFilter(_obj, _event);
  }

  public: std::function<void (QEvent *)> forwardEvent;
};
}
}
}

#endif
