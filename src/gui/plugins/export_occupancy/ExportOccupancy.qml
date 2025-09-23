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


ColumnLayout {
    id: root
    anchors.fill: parent
    anchors.margins: 20
    Layout.minimumWidth: 350
    Layout.minimumHeight: 800
    spacing: 10

    property int lidarSamples: 360
    property real lidarRange: 100.0
    property real lidarRangeResolution: 0.1
    property real lidarAngularResolution: 1.0
    property real distanceFromLidarToGround: 0.3
    property real occupancyGridCellResolution: 0.1
    property int occupancyGridNumberOfHorizontalCells: 100
    property int occupancyGridNumberOfVerticalCells: 100
    property string state: "configure"
    // Lidar Samples
    RowLayout {
        id: sampleRow
        Label { text: "Lidar Samples:" }
        TextField {
            id: lidarSamplesInput
            Layout.fillWidth: true
             validator: IntValidator { bottom: 1 }
            text: root.lidarSamples
            onEditingFinished: {
                root.lidarSamples = parseInt(text)
                if (isNaN(root.lidarSamples)) root.lidarSamples = 360.0 // Handle invalid input
            }
        }
    }

    // Lidar Range
    RowLayout {
        id: rangeRow
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
        id: rangeResolutionRow
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
        id: angularResolutionRow
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
        id: heightRow
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
        id: resolutionRow
        Label { text: "Cell Resolution (m):" }
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
        id: horizontalCellRow
        Label { text: "Number of Horizontal Cells:" }
        TextField {
            id: occupancyGridHorizontalCellsInput
            Layout.fillWidth: true
            validator: IntValidator { bottom: 0 }
            text:  root.occupancyGridNumberOfHorizontalCells.toFixed(2)
            onEditingFinished: {
                root.occupancyGridNumberOfHorizontalCells = parseInt(text)
                if (isNaN(root. root.occupancyGridNumberOfHorizontalCells))root.occupancyGridNumberOfHorizontalCells = 0
            }
        }
    }

    RowLayout {
        id: verticalCellRow
        Label { text: "Number of Vertical Cells:" }
        TextField {
            id: occupancyGridVerticalCellsInput
            Layout.fillWidth: true
            validator: IntValidator { bottom: 0 }
            text:  root.occupancyGridNumberOfVerticalCells.toFixed(2)
            onEditingFinished: {
                root.occupancyGridNumberOfVerticalCells = parseInt(text)
                if (isNaN(root. root.occupancyGridNumberOfVerticalCells)) root. root.occupancyGridNumberOfVerticalCells = 0
            }
        }
    }

    // Example of how you might use the properties (e.g., a button to print values)
    Button {
        id: startButton
        text: "Add Probe"
        onClicked: {
            if (startButton.text === "Add Probe") {
                sampleRow.visible =  false;
                rangeRow.visible = false;
                rangeResolutionRow.visible = false;
                angularResolutionRow.visible = false;
                horizontalCellRow.visible = false;
                verticalCellRow.visible = false;
                resolutionRow.visible = false;
                heightRow.visible = false;
                exportOccupancy.StartExport(
                    root.lidarSamples, root.lidarRange, root.lidarRangeResolution, root.lidarAngularResolution,
                    root.distanceFromLidarToGround, root.occupancyGridCellResolution, root.occupancyGridNumberOfHorizontalCells,
                    root.occupancyGridNumberOfVerticalCells);
                startButton.text = "Start Scan";
            }
            else if (startButton.text === "Start Scan")
            {
                exportOccupancy.StartExploration();
                startButton.text = "Save Occupancy Map";
            }
        }
    }
}
