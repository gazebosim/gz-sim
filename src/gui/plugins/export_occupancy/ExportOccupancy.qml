/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import "qrc:/qml"

Item {
    id: root
    anchors.fill: parent

    property int lidarSamples: 640
    property real lidarRange: 9.0
    property real lidarRangeResolution: 0.01
    property real lidarAngularResolution: 0.0
    property real distanceFromLidarToGround: 0.0
    property real occupancyGridCellResolution: 0.1
    property int occupancyGridNumberOfHorizontalCells: 100
    property int occupancyGridNumberOfVerticalCells: 100

    ScrollView {
        anchors.fill: parent
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOn
        ScrollBar.vertical.policy: ScrollBar.AlwaysOff
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 10

            // Lidar Samples
            RowLayout {
                Label { text: "Lidar Samples:" }
                SpinBox {
                    id: lidarSamplesInput
                    Layout.fillWidth: true
                    from: 0
                    to: 100000 // A reasonable upper bound, adjust as needed
                    value: root.lidarSamples
                    onValueChanged: root.lidarSamples = value
                    stepSize: 1
                }
            }

            // Lidar Range
            RowLayout {
                Label { text: "Lidar Range (m):" }
                TextField {
                    id: lidarRangeInput
                    Layout.fillWidth: true
                    validator: DoubleValidator { bottom: 0.0 }
                    text: root.lidarRange.toFixed(2)
                    onEditingFinished: {
                        root.lidarRange = parseFloat(text)
                        if (isNaN(root.lidarRange)) root.lidarRange = 0.0 // Handle invalid input
                    }
                }
            }

            // Lidar Range Resolution
            RowLayout {
                Label { text: "Lidar Range Resolution (m):" }
                TextField {
                    id: lidarRangeResolutionInput
                    Layout.fillWidth: true
                    validator: DoubleValidator { bottom: 0.0 }
                    text: root.lidarRangeResolution.toFixed(3)
                    onEditingFinished: {
                        root.lidarRangeResolution = parseFloat(text)
                        if (isNaN(root.lidarRangeResolution)) root.lidarRangeResolution = 0.0
                    }
                }
            }

            // Lidar Angular Resolution
            RowLayout {
                Label { text: "Lidar Angular Resolution (degrees):" }
                TextField {
                    id: lidarAngularResolutionInput
                    Layout.fillWidth: true
                    validator: DoubleValidator { bottom: 0.0; top: 360.0 }
                    text: root.lidarAngularResolution.toFixed(2)
                    onEditingFinished: {
                        root.lidarAngularResolution = parseFloat(text)
                        if (isNaN(root.lidarAngularResolution)) root.lidarAngularResolution = 0.0
                    }
                }
            }

            // Distance from Lidar to Ground
            RowLayout {
                Label { text: "Distance from Lidar to Ground (m):" }
                TextField {
                    id: distanceFromLidarToGroundInput
                    Layout.fillWidth: true
                    validator: DoubleValidator { bottom: 0.0 }
                    text: root.distanceFromLidarToGround.toFixed(2)
                    onEditingFinished: {
                        root.distanceFromLidarToGround = parseFloat(text)
                        if (isNaN(root.distanceFromLidarToGround)) root.distanceFromLidarToGround = 0.0
                    }
                }
            }

            // Occupancy Grid Cell Resolution
            RowLayout {
                Label { text: "Occupancy Grid Cell Resolution (m):" }
                TextField {
                    id: occupancyGridCellResolutionInput
                    Layout.fillWidth: true
                    validator: DoubleValidator { bottom: 0.0 }
                    text: root.occupancyGridCellResolution.toFixed(2)
                    onEditingFinished: {
                        root.occupancyGridCellResolution = parseFloat(text)
                        if (isNaN(root.occupancyGridCellResolution)) root.occupancyGridCellResolution = 0.0
                    }
                }
            }

            // Occupancy Grid Number of Horizontal Cells
            RowLayout {
                Label { text: "Occupancy Grid Number of Horizontal Cells:" }
                SpinBox {
                    id: occupancyGridHorizontalCellsInput
                    Layout.fillWidth: true
                    from: 0
                    to: 10000 // Adjust upper bound as needed
                    value: root.occupancyGridNumberOfHorizontalCells
                    onValueChanged: root.occupancyGridNumberOfHorizontalCells = value
                    stepSize: 1
                }
            }

            RowLayout {
                Label { text: "Occupancy Grid Number of Vertical Cells:" }
                SpinBox {
                    id: occupancyGridVerticalCellsInput
                    Layout.fillWidth: true
                    from: 0
                    to: 10000 // Adjust upper bound as needed
                    value: root.occupancyGridNumberOfVerticalCells
                    onValueChanged: root.occupancyGridNumberOfVerticalCells = value
                    stepSize: 1
                }
            }

            // Example of how you might use the properties (e.g., a button to print values)
            Button {
                text: "Add Probe"
                onClicked: {
                    console.log("Lidar Samples:", root.lidarSamples)
                    console.log("Lidar Range:", root.lidarRange, "m")
                    console.log("Lidar Range Resolution:", root.lidarRangeResolution, "m")
                    console.log("Lidar Angular Resolution:", root.lidarAngularResolution, "degrees")
                    console.log("Distance from Lidar to Ground:", root.distanceFromLidarToGround, "m")
                    console.log("Occupancy Grid Cell Resolution:", root.occupancyGridCellResolution, "m")
                    console.log("Occupancy Grid Number of Horizontal Cells:", root.occupancyGridNumberOfHorizontalCells)
                    exportOccupancy.StartExport(
                        root.lidarSamples, root.lidarRange, root.lidarRangeResolution, root.lidarAngularResolution,
                        root.distanceFromLidarToGround, root.occupancyGridCellResolution, root.occupancyGridNumberOfHorizontalCells,
                        root.occupancyGridNumberOfVerticalCells);
                }
            }
        }
    }
}
