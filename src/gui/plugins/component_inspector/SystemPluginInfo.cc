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
#include <gz/msgs/plugin_v.pb.h>

#include <gz/common/Console.hh>
#include <gz/sim/components/SystemPluginInfo.hh>

#include "SystemPluginInfo.hh"
#include "ComponentInspector.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;
using namespace inspector;

/////////////////////////////////////////////////
SystemPluginInfo::SystemPluginInfo(ComponentInspector *_inspector)
{
  _inspector->Context()->setContextProperty("SystemPluginInfoImpl", this);
  this->inspector = _inspector;

  this->inspector->AddUpdateViewCb(components::SystemPluginInfo::typeId,
      std::bind(&SystemPluginInfo::UpdateView, this, std::placeholders::_1,
      std::placeholders::_2));
}

/////////////////////////////////////////////////
void SystemPluginInfo::UpdateView(const EntityComponentManager &_ecm,
    QStandardItem *_item)
{
  auto comp = _ecm.Component<components::SystemPluginInfo>(
      this->inspector->GetEntity());
  if (nullptr == _item || nullptr == comp)
    return;

  auto msg = comp->Data();

  _item->setData(QString("SystemPluginInfo"),
      ComponentsModel::RoleNames().key("dataType"));

  QList<QVariant> pluginList;
  for (int i = 0; i < msg.plugins().size(); ++i)
  {
    QList<QVariant> dataList;
    dataList.push_back(
        QVariant(QString::fromStdString(msg.plugins(i).name())));
    dataList.push_back(
        QVariant(QString::fromStdString(msg.plugins(i).filename())));
    dataList.push_back(
        QVariant(QString::fromStdString(msg.plugins(i).innerxml())));
    pluginList.push_back(dataList);
  }

  _item->setData(pluginList,
      ComponentsModel::RoleNames().key("data"));
}
