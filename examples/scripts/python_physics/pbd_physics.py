from gz.sim import Model, World, world_entity, Link
from gz.sim import components, ComponentState
import numpy as np
from enum import Enum
import sdformat as sdf
import gz.sim
import time
from pbd_broadphase import SpatialHashGrid

# PBD Constants
GRAVITY = np.array([0.0, 0.0, -9.81])
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
        self.profile_data = {
            'external': 0.0,
            'broadphase': 0.0,
            'constraints': 0.0,
            'updates': 0.0,
            'total': 0.0,
            'count': 0,
        }

    def configure(self, _entity, _sdf, _ecm, _event_mgr):
        print("PBDPhysics configure method called")
        
        # 1. Create PBD components for LINKS with geometry
        for link_entity, _ in _ecm.each([components.Link]):
            link = Link(link_entity)
            
            if link.valid(_ecm):
                parent_comp = _ecm.component(link_entity, components.ParentEntity)
                is_static = False
                parent_model = None
                model_pos = np.array([0.0, 0.0, 0.0])
                
                if parent_comp:
                    parent_model = parent_comp
                    is_static = Model(parent_model).static(_ecm)
                     
                    # Cache model position!
                    model_pose_comp = _ecm.component(parent_model, components.Pose)
                    if model_pose_comp:
                        p = model_pose_comp.pos()
                        model_pos = np.array([p.x(), p.y(), p.z()])
                
                inv_mass = 0.0 if is_static else 1.0

                shape_type = None
                dimensions = {}

                collisions = link.collisions(_ecm)
                if collisions:
                    collision_entity = collisions[0]
                    base_comp = _ecm.component(collision_entity, components.Geometry)
                    if base_comp:
                        geom_data = base_comp
                        if geom_data.type() == sdf.GeometryType.SPHERE:
                            shape_type = ShapeType.SPHERE.value
                            dimensions['radius'] = geom_data.sphere_shape().radius()
                        elif geom_data.type() == sdf.GeometryType.BOX:
                            shape_type = ShapeType.BOX.value
                            s = geom_data.box_shape().size()
                            dimensions['size'] = np.array([s.x(), s.y(), s.z()])
                        elif geom_data.type() == sdf.GeometryType.CYLINDER:
                            shape_type = ShapeType.CYLINDER.value
                            dimensions['radius'] = geom_data.cylinder_shape().radius()
                            dimensions['length'] = geom_data.cylinder_shape().length()

                if not shape_type:
                    continue

                initial_pose = link.world_pose(_ecm)
                if initial_pose:
                    pos = initial_pose.pos()
                    pos_np = np.array([pos.x(), pos.y(), pos.z()])
                    pbd_data = {
                        "position": pos_np,
                        "predicted_position": pos_np.copy(),
                        "velocity": np.array([0.0, 0.0, 0.0]),
                        "shape_type": shape_type,
                        "dimensions": dimensions,
                        "inv_mass": inv_mass,
                        "initial_pose": initial_pose,
                        "model_entity": int(parent_model) if parent_model else None,
                        "model_pos": model_pos # Cached!
                    }
                    self.pbd_entities[link_entity] = pbd_data
                    name_comp = _ecm.component(link_entity, components.Name)
                    name = name_comp if name_comp else str(link_entity)
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
                        p_pos = p_pose.pos()
                        c_pos = c_pose.pos()
                        p_pos_np = np.array([p_pos.x(), p_pos.y(), p_pos.z()])
                        c_pos_np = np.array([c_pos.x(), c_pos.y(), c_pos.z()])
                        
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
                            offset = c_pos_np - p_pos_np
                            constraint_entity = _ecm.create_entity()
                            self.constraints[constraint_entity] = {
                                "type": "fixed",
                                "entity1": int(parent_entity),
                                "entity2": int(child_entity),
                                "offset": offset
                            }
                            print(f"Created FIXED constraint between link {parent_name} and {child_name}")
                        else:
                            dist = np.linalg.norm(p_pos_np - c_pos_np)
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
        dist = np.linalg.norm(delta_pos)
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
        t_t0 = time.perf_counter()
        dt = _info.dt.total_seconds()
        if dt == 0:
            return

        # Phase 1: External Forces (Gravity)
        t_e0 = time.perf_counter()
        for entity, data in self.pbd_entities.items():            
            if data['inv_mass'] == 0:
                continue
            data['velocity'] += GRAVITY * dt
            data['predicted_position'] = data['position'] + data['velocity'] * dt
        self.profile_data['external'] += time.perf_counter() - t_e0
        
        # --- OPTIMIZATION: Broad-phase ONCE per step ---
        t_b0 = time.perf_counter()

        self.broadphase.clear()
        pbd_entities = {}
        for entity, data in self.pbd_entities.items():
            pbd_entities[int(entity)] = data
            
            pos = data['predicted_position']
            if data['shape_type'] == ShapeType.SPHERE.value:
                r = data['dimensions']['radius']
                aabb_min = pos - r
                aabb_max = pos + r
            elif data['shape_type'] == ShapeType.BOX.value:
                s = data['dimensions']['size']
                aabb_min = pos - s / 2
                aabb_max = pos + s / 2
            else:
                aabb_min = pos
                aabb_max = pos
                
            self.broadphase.add(entity, aabb_min, aabb_max)

        candidate_pairs = self.broadphase.get_candidate_pairs()
        self.profile_data['broadphase'] += time.perf_counter() - t_b0
        # -----------------------------------------------

        # Phase 2: Constraint Resolution
        t_c0 = time.perf_counter()
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
                        dist = np.linalg.norm(delta_pos)
                        
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

        self.profile_data['constraints'] += time.perf_counter() - t_c0

        # Phase 3: Update Positions and Velocities
        t_u0 = time.perf_counter()
        for entity, data in self.pbd_entities.items():
            if data['inv_mass'] == 0:
                continue
            data['velocity'] = (data['predicted_position'] - data['position']) / dt
            data['position'] = data['predicted_position']

        # Phase 4: Apply to gz-sim
        current_time = time.monotonic()
        if current_time - self.last_component_update_time >= 1.0 / 60.0:
            self.last_component_update_time = current_time
            # Group simulated links by model
            model_simulated_links = {}
            for link_entity, data in self.pbd_entities.items():
                model_entity = data["model_entity"]
                if model_entity:
                    if model_entity not in model_simulated_links:
                        model_simulated_links[model_entity] = []
                    model_simulated_links[model_entity].append(link_entity)

            model_pose_map = {}
            for entity, (_, model_pose_comp) in _ecm.each([components.Model, components.Pose]):
                model_pose_map[entity] = model_pose_comp
                
            link_pose_map = {}
            for entity, (link_pose_comp,) in _ecm.each([components.Pose]):
                link_pose_map[entity] = link_pose_comp
                
            for model_entity, links in model_simulated_links.items():
                model_pose_comp = model_pose_map.get(model_entity)
                if not model_pose_comp:
                    continue
                    
                # Find canonical link
                canonical_link = None
                for l in links:
                    if _ecm.component(l, components.CanonicalLink):
                        canonical_link = l
                        break
                if not canonical_link:
                    canonical_link = links[0]
                    
                canonical_data = self.pbd_entities.get(canonical_link)
                if not canonical_data:
                    continue
                p_C = canonical_data['position']
                
                # Canonical link relative pose in model frame (stored in link's Pose component)
                ref_pose_in_model = link_pose_map.get(canonical_link)
                if not ref_pose_in_model:
                    continue
                p_MC = ref_pose_in_model.pos()
                
                # Model orientation (we keep current rotation, as PBD doesn't simulate rot)
                q_M = model_pose_comp.rot()
                
                # Model world position: p_M = p_C - q_M * p_MC
                p_MC_world = q_M * gz.math.Vector3d(p_MC.x(), p_MC.y(), p_MC.z())
                p_M = p_C - np.array([p_MC_world.x(), p_MC_world.y(), p_MC_world.z()])
                
                # Update model position component
                model_pose_comp.pos().set(p_M[0], p_M[1], p_M[2])
                _ecm.set_changed(model_entity, components.Pose.type_id, ComponentState.PeriodicChange)
                
                # Update all other non-canonical links' relative positions
                for l in links:
                    if l == canonical_link:
                        continue
                    link_data = self.pbd_entities.get(l)
                    if not link_data:
                        continue
                    p_L = link_data['position']
                    
                    rel_pos_world = gz.math.Vector3d(p_L[0] - p_M[0], p_L[1] - p_M[1], p_L[2] - p_M[2])
                    rel_pos_model = q_M.inverse() * rel_pos_world
                    
                    link_pose_comp = link_pose_map.get(l)
                    if link_pose_comp:
                        link_pose_comp.pos().set(rel_pos_model.x(), rel_pos_model.y(), rel_pos_model.z())
                        _ecm.set_changed(l, components.Pose.type_id, ComponentState.PeriodicChange)
        self.profile_data['updates'] += time.perf_counter() - t_u0
        self.profile_data['total'] += time.perf_counter() - t_t0
        self.profile_data['count'] += 1
        self._check_profile()

    def _check_profile(self):
        if self.profile_data['count'] >= 100:
            count = self.profile_data['count']
            print(f"--- Profile (avg over {count} steps) ---")
            print(f"  External Forces: {self.profile_data['external']/count*1000:.3f} ms/step")
            print(f"  Broadphase: {self.profile_data['broadphase']/count*1000:.3f} ms/step")
            print(f"  Constraints: {self.profile_data['constraints']/count*1000:.3f} ms/step")
            print(f"  Updates: {self.profile_data['updates']/count*1000:.3f} ms/step")
            print(f"  Total: {self.profile_data['total']/count*1000:.3f} ms/step")
            # Reset
            self.profile_data['external'] = 0.0
            self.profile_data['broadphase'] = 0.0
            self.profile_data['updates'] = 0.0
            self.profile_data['constraints'] = 0.0
            self.profile_data['total'] = 0.0
            self.profile_data['count'] = 0
def get_system():
    return PBDPhysics()
