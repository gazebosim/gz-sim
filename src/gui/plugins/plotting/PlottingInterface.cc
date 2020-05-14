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
#include "PlottingInterface.hh"

PlottingInterface :: PlottingInterface(Transport *_transport) : QObject()
{
    this->transport = _transport;
}

PlottingInterface::~PlottingInterface()
{
}

Q_INVOKABLE void PlottingInterface :: setTopic(QString _topic)
{
    this->transport->SetTopic(_topic.toStdString());
}

Q_INVOKABLE void PlottingInterface :: subscribe(QModelIndex _index)
{
    this->transport->Subscribe(_index);
}

void PlottingInterface :: emitPlotting(int _chart, float _x , QVariant _y)
{
    emit plot(_chart , _x , _y);
}


