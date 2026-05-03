import unittest
import gz.sim

class EcmEachTest(unittest.TestCase):
    def test_each(self):
        # Register custom component
        comp_type = gz.sim.components.register_custom_component("EachTestComponent")
        
        ecm = gz.sim.EntityComponentManager()
        
        # Create entities
        e1 = ecm.create_entity()
        e2 = ecm.create_entity()
        e3 = ecm.create_entity()
        
        # Verify component() returns None if not exists
        self.assertIsNone(ecm.component(e1, comp_type))
        
        # Add components
        ecm.create_component(e1, comp_type, {"val": 1})
        ecm.create_component(e2, comp_type, {"val": 2})
        
        # e3 has no component
        
        # Iterate using each
        count = 0
        cursor_ids = set()
        
        for item in ecm.each([comp_type]):
            count += 1
            cursor_ids.add(id(item))
            
            if item.entity == e1:
                self.assertEqual(item.components[0]["val"], 1)
                # Modify data in place
                item.components[0]["val"] = 10
            elif item.entity == e2:
                self.assertEqual(item.components[0]["val"], 2)
                item.components[0]["val"] = 20
            else:
                self.fail("Unexpected entity found")
                
        self.assertEqual(count, 2)
        # Verify cursor reuse: there should only be 1 unique ID for the item object!
        self.assertEqual(len(cursor_ids), 1)
        
        # Verify modifications persisted
        w1_again = ecm.component(e1, comp_type)
        self.assertEqual(w1_again.data()["val"], 10)
        
        w2_again = ecm.component(e2, comp_type)
        self.assertEqual(w2_again.data()["val"], 20)
        
        print("All tests passed!")

    def test_each_static(self):
        # Use a statically registered component, e.g., Name
        name_type = gz.sim.components.Name
            
        ecm = gz.sim.EntityComponentManager()
        
        e1 = ecm.create_entity()
        e2 = ecm.create_entity()
        
        ecm.create_component(e1, name_type, "entity1")
        ecm.create_component(e2, name_type, "entity2")
        
        count = 0
        for item in ecm.each([name_type]):
            count += 1
            if item.entity == e1:
                self.assertEqual(item.components[0], "entity1")
            elif item.entity == e2:
                self.assertEqual(item.components[0], "entity2")
            else:
                self.fail("Unexpected entity found")
                
        self.assertEqual(count, 2)

if __name__ == '__main__':
    unittest.main()
