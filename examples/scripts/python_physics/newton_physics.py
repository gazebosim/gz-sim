import gz
from gz.sim import Model, World, Link, components, ComponentState
from gz.math import Vector3d, Pose3d
import sdformat as sdf
import time
import logging
import sys
import os

# Note: We assume 'newton' and 'warp' are available in the environment as per user instruction.
import newton
import warp as wp

class NewtonPhysics:
    def __init__(self):
        self.scene = None
        self.model = None
        self.solver = None
        self.state_0 = None
        self.state_1 = None
        self.control = None
        self.contacts = None
        self.viewer = None
        
        self.entity_map = {} # Map Gazebo link entity to Newton link index
        self.link_entities = [] # List of link entities in order of Newton indices
        self.newton_caches = {}
        self.ecm_update_rate = 0.0
        self.last_component_update_time = 0.0
        self.sim_dt = 0.001
        self.sim_time = 0.0
        self.sim_substeps = 1
        
        # Profiling data
        self.profile_data = {
            'solver': 0.0,
            'ecm': 0.0,
            'viewer': 0.0,
            'count': 0,
            'ecm_count': 0
        }

    def configure(self, _entity, _sdf, _ecm, _event_mgr):
        print("NewtonPhysics configure method called")
        
        # 1. Retrieve dt from Physics component on world entity
        physics_comp = _ecm.component(_entity, components.Physics)
        if physics_comp:
            self.sim_dt = physics_comp.data().max_step_size()
            print(f"Retrieved dt from Physics component: {self.sim_dt}")
        else:
            print(f"Physics component not found, defaulting dt to: {self.sim_dt}")
            
        self.ecm_update_rate = _sdf.get_double('ecm_update_rate', 0.0)[0]
        print(f"Newton ECM update rate: {self.ecm_update_rate} Hz")
        
        # Check for viewer enable in SDF
        show_viewer = False
        if _sdf.has_element('show_viewer'):
            show_viewer = _sdf.get_bool('show_viewer')
            print(f"Newton show_viewer from SDF: {show_viewer}")
            
        if show_viewer:
            try:
                self.viewer = newton.viewer.ViewerGL()
                print("Newton GL Viewer initialized.")
            except Exception as e:
                print(f"Failed to initialize Newton viewer: {e}")
                
        # Check for substeps in SDF
        if _sdf.has_element('substeps'):
            self.sim_substeps = int(_sdf.get_double('substeps', 1.0)[0])
            print(f"Newton substeps from SDF: {self.sim_substeps}")
        
        builder = newton.ModelBuilder()
        
        # 2. Parse scene geometry from ECM by iterating on Collision entities
        gazebo_links = []
        for entity, _ in _ecm.each([components.Link]):
            gazebo_links.append(entity)
            
        # Add links to Newton
        for link_entity in gazebo_links:
            # Try to read inertial properties from Gazebo
            inertial_comp = _ecm.component(link_entity, components.Inertial)
            mass = 0.0
            inertia = None
            lock_inertia = False
            
            if inertial_comp:
                try:
                    inertial_data = inertial_comp.data()
                    mass = inertial_data.mass()
                    moi = inertial_data.moi()
                    
                    # Try different ways to access matrix elements
                    try:
                        inertia = wp.mat33(
                            float(moi(0,0)), float(moi(0,1)), float(moi(0,2)),
                            float(moi(1,0)), float(moi(1,1)), float(moi(1,2)),
                            float(moi(2,0)), float(moi(2,1)), float(moi(2,2))
                        )
                        lock_inertia = True
                    except:
                        try:
                            inertia = wp.mat33(
                                float(moi[0,0]), float(moi[0,1]), float(moi[0,2]),
                                float(moi[1,0]), float(moi[1,1]), float(moi[1,2]),
                                float(moi[2,0]), float(moi[2,1]), float(moi[2,2])
                            )
                            lock_inertia = True
                        except:
                            print(f"Could not parse MOI matrix for link {link_entity}, letting Newton compute inertia.")
                            
                    if lock_inertia:
                        print(f"Using inertial properties from Gazebo for link {link_entity}: mass={mass}")
                except Exception as e:
                    print(f"Failed to extract inertial data for link {link_entity}: {e}")
            
            # Get initial world pose of link
            link_pose = gz.sim.world_pose(link_entity, _ecm)
            xform = wp.transform(
                p=wp.vec3(link_pose.pos().x(), link_pose.pos().y(), link_pose.pos().z()),
                q=wp.quat(link_pose.rot().x(), link_pose.rot().y(), link_pose.rot().z(), link_pose.rot().w())
            )
            
            newton_link = builder.add_link(xform=xform, mass=mass, inertia=inertia, lock_inertia=lock_inertia)
            self.entity_map[link_entity] = newton_link
            self.link_entities.append(link_entity)
            print(f"Added link {newton_link} (Gazebo entity {link_entity}) at world pose: {link_pose.pos().x()}, {link_pose.pos().y()}, {link_pose.pos().z()}")
            
        seen_links = set()
        for entity, (_, collision_local_pose, parent_link_entity, geom_data) in _ecm.each([components.Collision, components.Pose, components.ParentEntity, components.Geometry]):
            if not geom_data:
                continue
                
            if parent_link_entity not in self.entity_map:
                continue
                
            if parent_link_entity in seen_links:
                continue
            seen_links.add(parent_link_entity)
            
            newton_link = self.entity_map[parent_link_entity]
            
            # Get geometry dimensions
            if geom_data.type() == sdf.GeometryType.SPHERE:
                radius = geom_data.sphere_shape().radius()
                builder.add_shape_sphere(newton_link, radius=radius)
                print(f"Added Sphere to link {newton_link} (Gazebo entity {parent_link_entity})")
                
            elif geom_data.type() == sdf.GeometryType.BOX:
                size = geom_data.box_shape().size()
                builder.add_shape_box(newton_link, hx=size.x()/2.0, hy=size.y()/2.0, hz=size.z()/2.0)
                print(f"Added Box to link {newton_link} (Gazebo entity {parent_link_entity})")
                
            elif geom_data.type() == sdf.GeometryType.PLANE:
                # Compute world pose of collision
                link_world_pose = gz.sim.world_pose(parent_link_entity, _ecm)
                collision_world_pose = link_world_pose * collision_local_pose
                height = collision_world_pose.pos().z()
                builder.add_ground_plane(height=height)
                print(f"Added Ground Plane at height {height} for link {parent_link_entity}")
                
        # If no collision geometry found in ECM, add default ground plane to avoid crash
        if not seen_links:
            print("No collision geometry found in ECM, adding default ground plane.")
            builder.add_ground_plane()
        
        # 3. Parse Joints
        joints = []
        for joint_entity, _ in _ecm.each([components.Joint]):
            joints.append(joint_entity)
            
        newton_joints = []
        for joint_entity in joints:
            joint = gz.sim.Joint(joint_entity)
            parent_name = joint.parent_link_name(_ecm)
            child_name = joint.child_link_name(_ecm)
            
            parent_entity = None
            child_entity = None
            
            # Find entities by name
            for link_entity, (name_comp,) in _ecm.each([components.Name]):
                if name_comp == parent_name:
                    parent_entity = link_entity
                elif name_comp == child_name:
                    child_entity = link_entity
                    
            if child_entity not in self.entity_map:
                print(f"Child link {child_name} not found in entity map, skipping joint.")
                continue
                
            child_newton_link = self.entity_map[child_entity]
            
            parent_newton_link = -1
            if parent_name == "world":
                parent_newton_link = -1
            elif parent_entity in self.entity_map:
                parent_newton_link = self.entity_map[parent_entity]
            else:
                print(f"Parent link {parent_name} not found in entity map, skipping joint.")
                continue
                
            # Extract joint axis
            axis_vec = wp.vec3(0.0, 0.0, 1.0) # Default axis
            axis_comp = _ecm.component(joint_entity, components.JointAxis)
            if axis_comp:
                axis_data = axis_comp.data()
                try:
                    xyz = axis_data.xyz()
                    axis_vec = wp.vec3(xyz.x(), xyz.y(), xyz.z())
                    print(f"Extracted axis for joint {joint_entity}: {xyz.x()}, {xyz.y()}, {xyz.z()}")
                except Exception as e:
                    print(f"Failed to extract axis data: {e}, using default Z axis.")
            
            # Check joint type
            is_fixed = False
            try:
                j_type = joint.type(_ecm)
                if j_type and "FIXED" in str(j_type).upper():
                    is_fixed = True
            except:
                pass
                
            # Compute transforms
            c_pose = gz.sim.world_pose(child_entity, _ecm)
            
            if parent_newton_link == -1:
                parent_xform = wp.transform(
                    p=wp.vec3(c_pose.pos().x(), c_pose.pos().y(), c_pose.pos().z()),
                    q=wp.quat(c_pose.rot().x(), c_pose.rot().y(), c_pose.rot().z(), c_pose.rot().w())
                )
                child_xform = wp.transform(p=wp.vec3(0.0, 0.0, 0.0), q=wp.quat_identity())
            else:
                p_pose = gz.sim.world_pose(parent_entity, _ecm)
                j_in_p = p_pose.inverse() * c_pose
                
                parent_xform = wp.transform(
                    p=wp.vec3(j_in_p.pos().x(), j_in_p.pos().y(), j_in_p.pos().z()),
                    q=wp.quat(j_in_p.rot().x(), j_in_p.rot().y(), j_in_p.rot().z(), j_in_p.rot().w())
                )
                child_xform = wp.transform(p=wp.vec3(0.0, 0.0, 0.0), q=wp.quat_identity())
                
            # Add joint
            if is_fixed:
                j = builder.add_joint_fixed(
                    parent=parent_newton_link,
                    child=child_newton_link,
                    parent_xform=parent_xform,
                    child_xform=child_xform,
                )
                print(f"Added Fixed joint between {parent_newton_link} and {child_newton_link}")
            else:
                j = builder.add_joint_revolute(
                    parent=parent_newton_link,
                    child=child_newton_link,
                    axis=axis_vec,
                    parent_xform=parent_xform,
                    child_xform=child_xform,
                )
                print(f"Added Revolute joint between {parent_newton_link} and {child_newton_link}")
                
            newton_joints.append(j)

        # Create articulation if we have joints
        if newton_joints:
            builder.add_articulation(newton_joints, label="gazebo_model")
            
        # Finalize model
        self.model = builder.finalize()
        self.solver = newton.solvers.SolverXPBD(self.model)
        
        # Set gravity from Gazebo
        gravity_comp = _ecm.component(_entity, components.Gravity)
        if gravity_comp:
            try:
                g = gravity_comp.data()
                self.model.set_gravity((g.x(), g.y(), g.z()))
                print(f"Set Newton gravity from Gazebo: {g.x()}, {g.y()}, {g.z()}")
            except Exception as e:
                print(f"Failed to set gravity from Gazebo: {e}")
        
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.contacts()
        
        # Initialize forward kinematics
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)
        
        # Set model in viewer if enabled
        if self.viewer:
            self.viewer.set_model(self.model)
            
        # Cache reference links for pose updates
        model_links = {}
        for link_entity in self.entity_map.keys():
            parent_comp = _ecm.component(link_entity, components.ParentEntity)
            if parent_comp:
                model_entity = parent_comp.data()
                if model_entity not in model_links:
                    model_links[model_entity] = []
                model_links[model_entity].append(link_entity)
                
        self.newton_caches = {}
        for model_entity, links in model_links.items():
            ref_link = links[0] # Fallback
            for l in links:
                if _ecm.component(l, components.CanonicalLink):
                    ref_link = l
                    break
                    
            for l in links:
                self.newton_caches[l] = {
                    'ref_link_entity': ref_link,
                    'model_entity': model_entity
                }

    def update(self, _info, _ecm):
        # Update viewer if enabled (even when paused)
        t_v0 = time.perf_counter()
        if self.viewer:
            self.viewer.begin_frame(self.sim_time)
            self.viewer.log_state(self.state_0)
            self.viewer.log_contacts(self.contacts, self.state_0)
            self.viewer.end_frame()
        self.profile_data['viewer'] += time.perf_counter() - t_v0
            
        if _info.paused:
            return
            
        dt = _info.dt.total_seconds()
        if dt == 0:
            dt = self.sim_dt
            
        substep_dt = dt / self.sim_substeps
        
        # Step simulation multiple times for substeps
        t_s0 = time.perf_counter()
        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            
            if self.viewer:
                self.viewer.apply_forces(self.state_0)
                
            self.model.collide(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, substep_dt)
            
            # Swap states
            self.state_0, self.state_1 = self.state_1, self.state_0
            
            self.sim_time += substep_dt
        self.profile_data['solver'] += time.perf_counter() - t_s0
        
        # Rate limiting for ECM update
        should_update_ecm = True
        if self.ecm_update_rate > 0.0:
            current_time = time.monotonic()
            if current_time - self.last_component_update_time < 1.0 / self.ecm_update_rate:
                should_update_ecm = False
            else:
                self.last_component_update_time = current_time
                
        if not should_update_ecm:
            self.profile_data['count'] += 1
            self._check_profile()
            return
            
        # Update Gazebo poses
        t_e0 = time.perf_counter()
        try:
            body_q = self.state_0.body_q.numpy()
        except Exception as e:
            print(f"Failed to get body_q as numpy array: {e}")
            self.profile_data['count'] += 1
            self._check_profile()
            return
            
        model_pose_map = {}
        for entity, (_, model_pose_comp) in _ecm.each([components.Model, components.Pose]):
            model_pose_map[entity] = model_pose_comp
            
        for link_entity, (link_pose_comp,) in _ecm.each([components.Pose]):
            cache = self.newton_caches.get(link_entity)
            if not cache:
                continue
                
            ref_link_entity = cache['ref_link_entity']
            model_entity = cache['model_entity']
            
            if link_entity not in self.entity_map:
                continue
                
            link_index = self.entity_map[link_entity]
            ref_link_index = self.entity_map[ref_link_entity]
            
            if len(body_q.shape) == 2:
                link_pos = body_q[link_index][:3]
                link_quat = body_q[link_index][3:]
                
                ref_pos = body_q[ref_link_index][:3]
                ref_quat = body_q[ref_link_index][3:]
            else:
                base_idx = link_index * 7
                ref_idx = ref_link_index * 7
                
                if base_idx + 6 >= len(body_q) or ref_idx + 6 >= len(body_q):
                    continue
                    
                link_pos = body_q[base_idx:base_idx+3]
                link_quat = body_q[base_idx+3:base_idx+7]
                
                ref_pos = body_q[ref_idx:ref_idx+3]
                ref_quat = body_q[ref_idx+3:ref_idx+7]
            
            # Compute relative position for Gazebo
            rel_pos = (link_pos[0] - ref_pos[0], link_pos[1] - ref_pos[1], link_pos[2] - ref_pos[2])
            
            link_pose_comp.pos().set(rel_pos[0], rel_pos[1], rel_pos[2])
            
            if link_entity == ref_link_entity:
                model_pose_comp = model_pose_map.get(model_entity)
                if model_pose_comp:
                    model_pose_comp.pos().set(ref_pos[0], ref_pos[1], ref_pos[2])
                    model_pose_comp.rot().set(link_quat[3], link_quat[0], link_quat[1], link_quat[2])
                    _ecm.set_changed(model_entity, components.Pose.type_id, ComponentState.PeriodicChange)
                    
                link_pose_comp.rot().set(1.0, 0.0, 0.0, 0.0)
            else:
                link_pose_comp.rot().set(1.0, 0.0, 0.0, 0.0)
                
            _ecm.set_changed(link_entity, components.Pose.type_id, ComponentState.PeriodicChange)
            
        self.profile_data['ecm'] += time.perf_counter() - t_e0
        self.profile_data['ecm_count'] += 1
        self.profile_data['count'] += 1
        
        self._check_profile()

    def _check_profile(self):
        if self.profile_data['count'] >= 100:
            count = self.profile_data['count']
            ecm_count = self.profile_data['ecm_count']
            print(f"--- Profile (avg over {count} steps) ---")
            print(f"  Solver: {self.profile_data['solver']/count*1000:.3f} ms/step")
            if ecm_count > 0:
                print(f"  ECM:    {self.profile_data['ecm']/ecm_count*1000:.3f} ms/update (called {ecm_count} times)")
            else:
                print(f"  ECM:    N/A (not called)")
            print(f"  Viewer: {self.profile_data['viewer']/count*1000:.3f} ms/step")
            # Reset
            self.profile_data['solver'] = 0.0
            self.profile_data['ecm'] = 0.0
            self.profile_data['viewer'] = 0.0
            self.profile_data['count'] = 0
            self.profile_data['ecm_count'] = 0

def get_system():
    return NewtonPhysics()
