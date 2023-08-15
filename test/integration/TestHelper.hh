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
#ifndef GZ_GUI_TESTHELPER_HH_
#define GZ_GUI_TESTHELPER_HH_

#include <gz/gui/Application.hh>
#include <gz/gui/Export.hh>
#include <gz/gui/MainWindow.hh>

#ifndef _WIN32
#  define TestHelper_EXPORTS_API
#else
#  if (defined(TestHelper_EXPORTS))
#    define TestHelper_EXPORTS_API __declspec(dllexport)
#  else
#    define TestHelper_EXPORTS_API __declspec(dllimport)
#  endif
#endif

namespace gz
{
namespace sim
{
/// \brief
class TestHelper_EXPORTS_API TestHelper : public QObject
{
  Q_OBJECT

  /// \brief Constructor
  public: inline TestHelper()
  {
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->
      installEventFilter(this);
  }

  /// \brief Destructor
  public: ~TestHelper() = default;

  /// \brief Documentation inherited
  public: inline bool eventFilter(QObject *_obj, QEvent *_event) override
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

#endif
