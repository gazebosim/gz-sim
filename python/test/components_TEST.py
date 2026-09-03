#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

from gz.math import Pose3d, Temperature, Vector3d
from gz.sim import EntityComponentManager, components

# Skiplist of components that are not currently exposed to Python because
# their underlying data types do not have Python bindings or pybind11 type casters.
KNOWN_SKIPPED_COMPONENTS = {
    "Actuators",
    "AirPressureSensor",
    "AirSpeedSensor",
    "Altimeter",
    "BatteryPowerLoad",
    "ContactSensor",
    "ContactSensorData",
    "Environment",
    "ExternalWorldWrenchCmd",
    "ForceTorque",
    "Imu",
    "JointTransmittedWrench",
    "LogicalCamera",
    "LogPlaybackStatistics",
    "Magnetometer",
    "NavSat",
    "ParticleEmitterCmd",
    "SystemPluginInfo",
    "VisualPlugin",
    "WheelSlipCmd",
    "WrenchMeasured",
}


class TestComponents(unittest.TestCase):

    def test_component_proxy(self):
        """Test ComponentProxy properties, representation, equality, and hash."""
        self.assertEqual("Model", components.Model.name)
        self.assertNotEqual(0, components.Model.type_id)
        self.assertEqual("<gz.sim.components.Model>", repr(components.Model))

        # Direct instantiation prohibited
        with self.assertRaises(TypeError):
            components.ComponentProxy("Custom", 12345)

        # Equality, inequality, and rich comparison
        self.assertEqual(components.Model, components.Model)
        self.assertNotEqual(components.Model, components.Link)
        self.assertNotEqual(components.Model, 12345)
        self.assertNotEqual(components.Model, "Model")

        # Hashability in dictionary
        d = {components.Model: "model_val", components.Link: "link_val"}
        self.assertEqual("model_val", d[components.Model])
        self.assertNotIn(components.Pose, d)

        # Unknown attribute raises AttributeError
        with self.assertRaises(AttributeError):
            _ = components.NonExistentComponent123

    def test_tag_components_lifecycle(self):
        """Test tag/marker (NoData) component creation, query, and removal."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()

        self.assertFalse(ecm.entity_has_component_type(e, components.Model))
        self.assertIsNone(ecm.component(e, components.Model))

        # Create tag component
        ecm.create_component(e, components.Model)
        self.assertTrue(ecm.entity_has_component_type(e, components.Model))
        self.assertEqual(components.Model, ecm.component(e, components.Model))

        # Removal
        self.assertTrue(ecm.remove_component(e, components.Model))
        self.assertFalse(ecm.entity_has_component_type(e, components.Model))
        self.assertFalse(ecm.remove_component(e, components.Model))

    def test_data_components_and_reference_semantics(self):
        """Test data components, in-place reference mutation, and change marking."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()

        init_pose = Pose3d(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        ecm.create_component(e, components.Pose, init_pose)

        pose = ecm.component(e, components.Pose)
        self.assertAlmostEqual(1.0, pose.x())

        # In-place field mutation via reference semantics
        pose.set_x(42.0)
        self.assertAlmostEqual(42.0, ecm.component(e, components.Pose).x())

        # Set component data
        self.assertTrue(ecm.set_component_data(e, components.Pose, init_pose))
        self.assertAlmostEqual(1.0, ecm.component(e, components.Pose).x())
        ecm.set_changed(e, components.Pose)

    def test_primitive_and_container_components(self):
        """Test primitive and STL container components."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()

        # String: Name with compare flag
        ecm.create_component(e, components.Name, "test_name")
        self.assertEqual("test_name", ecm.component(e, components.Name))
        self.assertFalse(ecm.set_component_data(e, components.Name, "test_name"))
        self.assertTrue(ecm.set_component_data(e, components.Name, "new_name"))
        self.assertEqual("new_name", ecm.component(e, components.Name))

        # Double: LevelBuffer with compare=False
        ecm.create_component(e, components.LevelBuffer, 12.5)
        self.assertTrue(ecm.set_component_data(e, components.LevelBuffer, 12.5, False))

        # Vector of double: JointPosition
        ecm.create_component(e, components.JointPosition, [1.5, 2.5])
        self.assertEqual([1.5, 2.5], ecm.component(e, components.JointPosition))

        # Set of string: LevelEntityNames
        ecm.create_component(e, components.LevelEntityNames, {"a", "b"})
        self.assertEqual({"a", "b"}, ecm.component(e, components.LevelEntityNames))

    def test_internal_struct_components(self):
        """Test internal C++ struct components."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()

        # DetachableJointInfo
        joint_info = components.DetachableJointInfo()
        joint_info.parent_link = 10
        joint_info.child_link = 20
        joint_info.joint_type = "fixed"
        ecm.create_component(e, components.DetachableJoint, joint_info)
        self.assertEqual(joint_info, ecm.component(e, components.DetachableJoint))

        # In-place mutation
        ecm.component(e, components.DetachableJoint).parent_link = 30
        self.assertEqual(30, ecm.component(e, components.DetachableJoint).parent_link)

        # TemperatureRangeInfo
        temp_range = components.TemperatureRangeInfo()
        temp_range.min = Temperature(200.0)
        temp_range.max = Temperature(400.0)
        ecm.create_component(e, components.TemperatureRange, temp_range)
        self.assertEqual(temp_range, ecm.component(e, components.TemperatureRange))

        # RaycastDataInfo
        ray = components.RayInfo()
        ray.end = Vector3d(1.0, 0.0, 0.0)
        res = components.RaycastResultInfo()
        res.fraction = 0.5
        data = components.RaycastDataInfo()
        data.rays = [ray]
        data.results = [res]
        ecm.create_component(e, components.RaycastData, data)
        read_data = ecm.component(e, components.RaycastData)
        self.assertEqual(Vector3d(1.0, 0.0, 0.0), read_data.rays[0].end)
        self.assertEqual(0.5, read_data.results[0].fraction)

    def test_type_error_exceptions(self):
        """Test Python TypeError propagation on invalid types."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()

        # Incompatible data type
        with self.assertRaises(TypeError):
            ecm.create_component(e, components.Pose, "not_a_pose")

        # None data for data component
        with self.assertRaises(TypeError):
            ecm.set_component_data(e, components.Pose, None)

        # Omitting data for data component
        with self.assertRaises(TypeError):
            ecm.create_component(e, components.Pose)

        # Providing data for tag component
        with self.assertRaises(TypeError):
            ecm.create_component(e, components.Model, "unexpected_data")

        # Unregistered component
        with self.assertRaises(TypeError):
            ecm.create_component(e, components.Actuators, "data")

        # Non-existent entity
        with self.assertRaises(TypeError):
            ecm.create_component(999999, components.Model)

        # Passing non-ComponentProxy arguments
        with self.assertRaises(TypeError):
            ecm.component(e, "invalid_type")

        with self.assertRaises(TypeError):
            ecm.entity_has_component_type(e, 12345)

        with self.assertRaises(TypeError):
            ecm.remove_component(e, 12345)

        with self.assertRaises(TypeError):
            ecm.set_changed(e, components.Pose.type_id)

    def test_entity_hierarchy_api(self):
        """Test ECM entity creation, existence, and hierarchy."""
        ecm = EntityComponentManager()
        parent = ecm.create_entity()
        child = ecm.create_entity()

        self.assertTrue(ecm.has_entity(parent))
        self.assertTrue(ecm.has_entity(child))
        self.assertFalse(ecm.has_entity(999999))

        self.assertEqual(0, ecm.parent_entity(child))
        ecm.set_parent_entity(child, parent)
        self.assertEqual(parent, ecm.parent_entity(child))

    def test_component_registration_parity(self):
        """Test full parity between C++ ComponentFactory and Python bindings."""
        all_components = components.all_factory_components()
        self.assertGreater(len(all_components), 0)

        for comp in all_components:
            if comp.name in KNOWN_SKIPPED_COMPONENTS:
                self.assertFalse(
                    components.has_python_bindings(comp),
                    f"Component '{comp.name}' is in KNOWN_SKIPPED_COMPONENTS but has bindings.")
            else:
                self.assertTrue(
                    components.has_python_bindings(comp),
                    f"Component '{comp.name}' is missing Python bindings.")


if __name__ == "__main__":
    unittest.main()
