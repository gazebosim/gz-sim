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
#ifndef GZ_SIM_GUI_COMPONENTINSPECTOR_SYSTEMINFO_HH_
#define GZ_SIM_GUI_COMPONENTINSPECTOR_SYSTEMINFO_HH_

#include "gz/sim/EntityComponentManager.hh"

#include <QObject>
#include <QStandardItem>

namespace gz
{
namespace sim
{
class ComponentInspector;
namespace inspector
{
  /// \brief A class that handles SystemPluginInfo components.
  class SystemPluginInfo : public QObject
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _inspector The component inspector.
    public: explicit SystemPluginInfo(ComponentInspector *_inspector);

    /// \brief Callback when there are ECM updates.
    /// \param[in] _ecm Immutable reference to the ECM.
    /// \param[in] _item Item to update.
    public: void UpdateView(const EntityComponentManager &_ecm,
        QStandardItem *_item);

    /// \brief Pointer to the component inspector. This is used to add
    /// callbacks.
    private: ComponentInspector *inspector{nullptr};
  };
}
}
}
#endif
