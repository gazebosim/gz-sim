from gz.sim import Model, World, world_entity, Link
from gz.sim import components, ComponentState
from gz.math import Vector3d, Pose3d
from enum import Enum
import sdformat as sdf
import gz.sim
import time
from pbd_broadphase import SpatialHashGrid

# PBD Constants
GRAVITY = Vector3d(0, 0, -9.81)
PBD_ITERATIONS = 5 # Reduced from 10

class ShapeType(Enum):
    SPHERE = 1
    BOX = 2
    CYLINDER = 3

class PBDPhysics:
    def __init__(self):
        self.broadphase = SpatialHashGrid(cell_size=2.0)
        self.pbd_entities = {}
        self.constraints = {}
        self.last_component_update_time = 0.0

    def configure(self, _entity, _sdf, _ecm, _event_mgr):
        print("PBDPhysics configure method called")
        
        # 1. Create PBD components for LINKS with geometry
        for link_entity, _ in _ecm.each([components.Link]):
            link = Link(link_entity)
            
            if link.valid(_ecm):
                parent_comp = _ecm.component(link_entity, components.ParentEntity)
                is_static = False
                parent_model = None
                model_pos = Vector3d(0, 0, 0)
                
                if parent_comp:
                     parent_model = parent_comp.data()
                     is_static = Model(parent_model).static(_ecm)
                     
                     # Cache model position!
                     model_pose_comp = _ecm.component(parent_model, components.Pose)
                     if model_pose_comp:
                         model_pos = model_pose_comp.data().pos()
                
                inv_mass = 0.0 if is_static else 1.0

                shape_type = None
                dimensions = {}

                collisions = link.collisions(_ecm)
                if collisions:
                    collision_entity = collisions[0]
                    base_comp = _ecm.component(collision_entity, components.Geometry)
                    if base_comp and base_comp.data():
                        geom_data = base_comp.data()
                        if geom_data.type() == sdf.GeometryType.SPHERE:
                            shape_type = ShapeType.SPHERE.value
                            dimensions['radius'] = geom_data.sphere_shape().radius()
                        elif geom_data.type() == sdf.GeometryType.BOX:
                            shape_type = ShapeType.BOX.value
                            dimensions['size'] = geom_data.box_shape().size()
                        elif geom_data.type() == sdf.GeometryType.CYLINDER:
                            shape_type = ShapeType.CYLINDER.value
                            dimensions['radius'] = geom_data.cylinder_shape().radius()
                            dimensions['length'] = geom_data.cylinder_shape().length()

                if not shape_type:
                    continue

                initial_pose = link.world_pose(_ecm)
                if initial_pose:
                    pbd_data = {
                        "position": initial_pose.pos(),
                        "predicted_position": initial_pose.pos(),
                        "velocity": Vector3d(0, 0, 0),
                        "shape_type": shape_type,
                        "dimensions": dimensions,
                        "inv_mass": inv_mass,
                        "initial_pose": initial_pose,
                        "model_entity": int(parent_model) if parent_model else None,
                        "model_pos": model_pos # Cached!
                    }
                    self.pbd_entities[link_entity] = pbd_data
                    name_comp = _ecm.component(link_entity, components.Name)
                    name = name_comp.data() if name_comp else str(link_entity)
                    print(f"Created PBD data for link: {name}")

        # 2. Create Distance and Fixed Constraints from Joints
        for joint_entity, _ in _ecm.each([components.Joint]):
            joint = gz.sim.Joint(joint_entity)
            
            parent_name = joint.parent_link_name(_ecm)
            child_name = joint.child_link_name(_ecm)
            
            if parent_name and child_name:
                if parent_name == "world":
                    for link_entity, (name_comp,) in _ecm.each([components.Name]):
                        if name_comp == child_name:
                            comp_data = self.pbd_entities.get(link_entity)
                            if comp_data:
                                comp_data['inv_mass'] = 0.0
                                print(f"Made link {child_name} static because it is attached to world.")
                    continue
                    
                parent_entity = None
                child_entity = None
                
                for link_entity, (name,) in _ecm.each([components.Name]):
                    if name == parent_name:
                        parent_entity = link_entity
                    elif name == child_name:
                        child_entity = link_entity
                        
                if parent_entity and child_entity:
                    p_pose = Link(parent_entity).world_pose(_ecm)
                    c_pose = Link(child_entity).world_pose(_ecm)
                    if p_pose and c_pose:
                        is_fixed = False
                        try:
                            j_type = joint.type(_ecm)
                            if j_type and "FIXED" in str(j_type).upper():
                                is_fixed = True
                        except:
                            pass
                            
                        if not is_fixed:
                            j_name = joint.name(_ecm)
                            if j_name and "fixed" in j_name.lower():
                                is_fixed = True
                                
                        if is_fixed:
                            offset = c_pose.pos() - p_pose.pos()
                            constraint_entity = _ecm.create_entity()
                            self.constraints[constraint_entity] = {
                                "type": "fixed",
                                "entity1": int(parent_entity),
                                "entity2": int(child_entity),
                                "offset": offset
                            }
                            print(f"Created FIXED constraint between link {parent_name} and {child_name}")
                        else:
                            dist = (p_pose.pos() - c_pose.pos()).length()
                            constraint_entity = _ecm.create_entity()
                            self.constraints[constraint_entity] = {
                                "type": "distance",
                                "entity1": int(parent_entity),
                                "entity2": int(child_entity),
                                "distance": dist
                            }
                            print(f"Created DISTANCE constraint between link {parent_name} and {child_name}")

    def resolve_sphere_sphere(self, obj1, obj2):
        delta_pos = obj1['predicted_position'] - obj2['predicted_position']
        dist = delta_pos.length()
        min_dist = obj1['dimensions']['radius'] + obj2['dimensions']['radius']

        if dist < min_dist and dist > 1e-6:
            penetration = min_dist - dist
            normal = delta_pos / dist
            
            total_inv_mass = obj1['inv_mass'] + obj2['inv_mass']
            if total_inv_mass == 0:
                return

            correction_vector = normal * penetration / total_inv_mass
            obj1['predicted_position'] += correction_vector * obj1['inv_mass']
            obj2['predicted_position'] -= correction_vector * obj2['inv_mass']

    def update(self, _info, _ecm):
        dt = _info.dt.total_seconds()
        if dt == 0:
            return

        # Phase 1: External Forces (Gravity)
        for entity, data in self.pbd_entities.items():            
            if data['inv_mass'] == 0:
                continue
            data['velocity'] += GRAVITY * dt
            data['predicted_position'] = data['position'] + data['velocity'] * dt
        
        # --- OPTIMIZATION: Broad-phase ONCE per step ---
        self.broadphase.clear()
        pbd_entities = {}
        for entity, data in self.pbd_entities.items():
            pbd_entities[int(entity)] = data
            
            pos = data['predicted_position']
            if data['shape_type'] == ShapeType.SPHERE.value:
                r = data['dimensions']['radius']
                aabb_min = Vector3d(pos.x() - r, pos.y() - r, pos.z() - r)
                aabb_max = Vector3d(pos.x() + r, pos.y() + r, pos.z() + r)
            elif data['shape_type'] == ShapeType.BOX.value:
                s = data['dimensions']['size']
                aabb_min = Vector3d(pos.x() - s.x()/2, pos.y() - s.y()/2, pos.z() - s.z()/2)
                aabb_max = Vector3d(pos.x() + s.x()/2, pos.y() + s.y()/2, pos.z() + s.z()/2)
            else:
                aabb_min = pos
                aabb_max = pos
                
            self.broadphase.add(entity, aabb_min, aabb_max)

        candidate_pairs = self.broadphase.get_candidate_pairs()
        # -----------------------------------------------

        # Phase 2: Constraint Resolution
        for i in range(PBD_ITERATIONS):
            # Ground Collision
            for entity, data in self.pbd_entities.items():
                if data['shape_type'] == ShapeType.SPHERE.value:
                    radius = data['dimensions']['radius']
                    if data['predicted_position'][2] < radius:
                        data['predicted_position'][2] = radius

            # Object-Object Collision using cached candidates
            for e1, e2 in candidate_pairs:
                obj1 = pbd_entities[e1]
                obj2 = pbd_entities[e2]
                
                if obj1['inv_mass'] == 0 and obj2['inv_mass'] == 0:
                    continue
                    
                if obj1['shape_type'] == ShapeType.SPHERE.value and obj2['shape_type'] == ShapeType.SPHERE.value:
                    self.resolve_sphere_sphere(obj1, obj2)

            # Distance and Fixed Constraints
            for entity, c_data in self.constraints.items():
                e1 = c_data['entity1']
                e2 = c_data['entity2']
                
                if e1 in pbd_entities and e2 in pbd_entities:
                    obj1 = pbd_entities[e1]
                    obj2 = pbd_entities[e2]
                    
                    total_inv_mass = obj1['inv_mass'] + obj2['inv_mass']
                    if total_inv_mass == 0:
                        continue
                        
                    if c_data.get('type', 'distance') == 'distance':
                        desired_dist = c_data['distance']
                        delta_pos = obj1['predicted_position'] - obj2['predicted_position']
                        dist = delta_pos.length()
                        
                        if dist > 1e-6:
                            penetration = dist - desired_dist
                            normal = delta_pos / dist
                            
                            correction_vector = normal * penetration / total_inv_mass
                            obj1['predicted_position'] -= correction_vector * obj1['inv_mass']
                            obj2['predicted_position'] += correction_vector * obj2['inv_mass']
                    elif c_data['type'] == 'fixed':
                        offset = c_data['offset']
                        current_offset = obj2['predicted_position'] - obj1['predicted_position']
                        error = current_offset - offset
                        
                        correction = error / total_inv_mass
                        obj1['predicted_position'] += correction * obj1['inv_mass']
                        obj2['predicted_position'] -= correction * obj2['inv_mass']

        # Phase 3: Update Positions and Velocities
        for entity, data in self.pbd_entities.items():
            if data['inv_mass'] == 0:
                continue
            data['velocity'] = (data['predicted_position'] - data['position']) / dt
            data['position'] = data['predicted_position']

        # Phase 4: Apply to gz-sim
        current_time = time.monotonic()
        if current_time - self.last_component_update_time >= 1.0 / 60.0:
            self.last_component_update_time = current_time
            for entity, (world_pose,) in _ecm.each([components.Pose]):
                data = self.pbd_entities.get(entity)
                if not data or data['inv_mass'] == 0:
                    continue
                
                # Use cached model_pos!
                relative_pos = data['position'] - data.get('model_pos', Vector3d(0,0,0))
                        
                world_pose.pos().set(relative_pos.x(), relative_pos.y(), relative_pos.z())
                _ecm.set_changed(entity, components.Pose.type_id, ComponentState.PeriodicChange)
def get_system():
    return PBDPhysics()
