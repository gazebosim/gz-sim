import unittest
import gz.sim

class EcmEachTest(unittest.TestCase):
    def test_each(self):
        name_type = gz.sim.components.Name
        parent_type = gz.sim.components.ParentEntity
        
        ecm = gz.sim.EntityComponentManager()
        
        # Create entities
        e1 = ecm.create_entity()
        e2 = ecm.create_entity()
        e3 = ecm.create_entity()
        
        # Verify component() returns None if not exists
        self.assertIsNone(ecm.component(e1, name_type))
        
        # Add components
        ecm.create_component(e1, name_type, "entity1")
        ecm.create_component(e1, parent_type, e2)
        
        ecm.create_component(e3, name_type, "entity3")
        
        # e2 has no Name component, only e1 and e3 do.
        # e1 has both Name and ParentEntity components.
        
        # Test iteration with single component: Name
        count = 0
        names = {}
        for entity, (name_val,) in ecm.each([name_type]):
            count += 1
            names[entity] = name_val
            
        self.assertEqual(count, 2)
        self.assertEqual(names[e1], "entity1")
        self.assertEqual(names[e3], "entity3")
        self.assertNotIn(e2, names)
        
        # Test iteration with multiple components: Name and ParentEntity
        count = 0
        matches = []
        for entity, (name_val, parent_val) in ecm.each([name_type, parent_type]):
            count += 1
            matches.append((entity, name_val, parent_val))

        self.assertEqual(count, 1)
        self.assertEqual(matches[0], (e1, "entity1", e2))
        
        # Test modifying component values
        # create_component just sets it, but we can also use set_component_data
        changed = ecm.set_component_data(e1, name_type, "updated_entity1")
        self.assertTrue(changed)
        
        # Test modifying component values again with the same data
        changed = ecm.set_component_data(e1, name_type, "updated_entity1")
        self.assertFalse(changed)
        
        # Test that skipping comparison always returns true
        changed = ecm.set_component_data(e1, name_type, "updated_entity1", compare=False)
        self.assertTrue(changed)

        val = ecm.component(e1, name_type)
        self.assertEqual(val, "updated_entity1")
        
        print("test_each passed!")

    def test_each_static(self):
        # Use a statically registered component, e.g., Name
        name_type = gz.sim.components.Name
            
        ecm = gz.sim.EntityComponentManager()
        
        e1 = ecm.create_entity()
        e2 = ecm.create_entity()
        
        ecm.create_component(e1, name_type, "entity1")
        ecm.create_component(e2, name_type, "entity2")
        
        count = 0
        for entity, (name_val,) in ecm.each([name_type]):
            count += 1
            if entity == e1:
                self.assertEqual(name_val, "entity1")
            elif entity == e2:
                self.assertEqual(name_val, "entity2")
            else:
                self.fail("Unexpected entity found")
                
        self.assertEqual(count, 2)
        print("test_each_static passed!")

    def test_entity_lifecycle_apis(self):
        name_type = gz.sim.components.Name
        ecm = gz.sim.EntityComponentManager()

        # 1. Create entities and test has_entity
        e1 = ecm.create_entity()
        e2 = ecm.create_entity()
        self.assertTrue(ecm.has_entity(e1))
        self.assertTrue(ecm.has_entity(e2))

        # 2. Test parent_entity
        self.assertEqual(ecm.parent_entity(e1), 0)

        # 3. Test set_parent_entity and parent_entity
        self.assertTrue(ecm.set_parent_entity(e1, e2))
        self.assertEqual(ecm.parent_entity(e1), e2)

        # 4. Test children
        child_list = ecm.children(e2)
        self.assertEqual(len(child_list), 1)
        self.assertEqual(child_list[0], e1)

        # 5. Test has_component_type
        self.assertFalse(ecm.has_component_type(e1, name_type))
        ecm.create_component(e1, name_type, "child_entity")
        self.assertTrue(ecm.has_component_type(e1, name_type))

        # 6. Test each_new
        new_items = ecm.each_new([name_type])
        self.assertEqual(len(new_items), 1)
        self.assertEqual(new_items[0][0], e1)
        self.assertEqual(new_items[0][1][0], "child_entity")

        # 7. Test remove_component and has_component_type
        self.assertTrue(ecm.remove_component(e1, name_type))
        self.assertFalse(ecm.has_component_type(e1, name_type))

        # 8. Test request_remove_entity and each_removed
        ecm.create_component(e1, name_type, "to_be_deleted")
        ecm.request_remove_entity(e1)
        removed_items = ecm.each_removed([name_type])
        self.assertEqual(len(removed_items), 1)
        self.assertEqual(removed_items[0][0], e1)
        self.assertEqual(removed_items[0][1][0], "to_be_deleted")
        print("test_entity_lifecycle_apis passed!")

if __name__ == '__main__':
    unittest.main()
