import unittest
import gz.sim

class CustomComponentTest(unittest.TestCase):
    def test_custom_component(self):
        # Register a custom component
        comp_type = gz.sim.components.register_custom_component("MyCustomComponent")
        self.assertEqual(comp_type.name, "MyCustomComponent")
        self.assertNotEqual(comp_type.type_id, 0)
        
        # Create an ECM
        ecm = gz.sim.EntityComponentManager()
        
        # Create an entity
        entity = ecm.create_entity()
        
        # Create data
        data = {"key": "value", "number": 42}
        
        # Use ComponentDataWrapper to set data
        wrapper = ecm.component(entity, comp_type)
        self.assertIsNotNone(wrapper)
        
        wrapper.set_data(data)
        
        # Retrieve data
        retrieved_data = wrapper.data()
        self.assertEqual(retrieved_data, data)
        
        # Modify data in wrapper
        new_data = {"key": "new_value", "number": 43}
        wrapper.set_data(new_data)
        
        # Verify modification
        retrieved_data = wrapper.data()
        self.assertEqual(retrieved_data, new_data)
        
        # Test cloning behavior (deep copy)
        entity2 = ecm.create_entity()
        wrapper2 = ecm.component(entity2, comp_type)
        self.assertIsNotNone(wrapper2)
        
        # Set data on entity2 using retrieved_data from entity1
        wrapper2.set_data(retrieved_data)
        
        # Now entity2 should have a DEEP COPY of the data!
        retrieved_data2 = wrapper2.data()
        self.assertEqual(retrieved_data2, retrieved_data)
        
        # Modify retrieved_data2
        retrieved_data2["key"] = "modified_in_2"
        
        # Verify retrieved_data (from entity1) is NOT modified!
        retrieved_data_again = wrapper.data()
        self.assertEqual(retrieved_data_again["key"], "new_value")
        
        print("All tests passed!")

if __name__ == '__main__':
    unittest.main()
