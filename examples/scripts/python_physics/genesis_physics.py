import genesis as gs
import gz
from gz.sim import Model, World, Link, components, ComponentState
from gz.math import Vector3d, Pose3d
import sdformat as sdf
from genesis.options.solvers import SimOptions
from genesis.options.profiling import ProfilingOptions
import time
import logging
import numpy as np
import sys
import os
import shutil

def to_numpy(val):
    if hasattr(val, 'cpu'):
        return val.cpu().numpy()
    return np.array(val)

class GenesisPhysics:
    def __init__(self):
        self.scene = None
        self.entity_map = {} # Map Gazebo link entity to Genesis entity
        self.genesis_caches = {}
        self.ecm_update_rate = 0.0
        self.last_component_update_time = 0.0
        self.profile_data = {
            'solver': 0.0,
            'total': 0.0,
            'count': 0,
        }

    def configure(self, _entity, _sdf, _ecm, _event_mgr):
        print("GenesisPhysics configure method called")
        
        # 1. Retrieve dt from Physics component on world entity
        physics_comp = _ecm.component(_entity, components.Physics)
        if physics_comp:
            dt = physics_comp.max_step_size()
            print(f"Retrieved dt from Physics component: {dt}")
        else:
            dt = 0.001 # Fallback if not found
            print(f"Physics component not found, defaulting dt to: {dt}")
            
        # 2. Initialize Genesis
        backend_str = "cpu"
        if _sdf.has_element('backend'):
            backend_str = _sdf.get_string('backend')
            
        print(f"Genesis using backend requested: {backend_str}")
        
        self.ecm_update_rate = _sdf.get_double('ecm_update_rate', 0.0)[0]
        print(f"Genesis ECM update rate: {self.ecm_update_rate} Hz")
        
        backend = gs.gpu if backend_str == "gpu" else gs.cpu
        detected_python = shutil.which('python3') or 'python3'
            
        print(f"Genesis using python: {detected_python}")
            
        orig_exe = sys.executable
        sys.executable = detected_python
        try:
            gs.init(backend=backend, logging_level=logging.INFO, performance_mode=True)
        finally:
            sys.executable = orig_exe
        
        sim_options = SimOptions(dt=dt)
        profiling_options = ProfilingOptions(show_FPS=False)
        self.scene = gs.Scene(sim_options=sim_options, show_viewer=False, profiling_options=profiling_options)
        
        # 3. Parse scene geometry from ECM by iterating on Collision entities
        seen_links = set()
        for entity, (_, collision_local_pose, parent_link_entity, geom_data) in _ecm.each([components.Collision, components.Pose, components.ParentEntity, components.Geometry]):
            collision_entity = entity
            
            if not geom_data:
                continue
                
            if parent_link_entity in seen_links:
                name_comp = _ecm.component(parent_link_entity, components.Name)
                name = name_comp if name_comp else str(parent_link_entity)
                print(f"Link {name} has multiple collisions. Genesis only supports one per link in this mode. Ignoring this one.")
                continue
                
            seen_links.add(parent_link_entity)
            
            # Compute world pose of collision using the bound world_pose function
            link_world_pose = gz.sim.world_pose(parent_link_entity, _ecm)
            
            # Compute world pose of collision
            collision_world_pose = link_world_pose * collision_local_pose
            pos = (collision_world_pose.pos().x(), collision_world_pose.pos().y(), collision_world_pose.pos().z())
            
            name_comp = _ecm.component(parent_link_entity, components.Name)
            name = name_comp if name_comp else str(parent_link_entity)
            
            if geom_data.type() == sdf.GeometryType.SPHERE:
                radius = geom_data.sphere_shape().radius()
                print(f"Adding Sphere for link {name}: pos={pos}, radius={radius}")
                gen_entity = self.scene.add_entity(gs.morphs.Sphere(pos=pos, radius=radius))
                self.entity_map[parent_link_entity] = gen_entity
                
            elif geom_data.type() == sdf.GeometryType.BOX:
                size = geom_data.box_shape().size()
                size_tuple = (size.x(), size.y(), size.z())
                print(f"Adding Box for link {name}: pos={pos}, size={size_tuple}")
                gen_entity = self.scene.add_entity(gs.morphs.Box(pos=pos, size=size_tuple))
                self.entity_map[parent_link_entity] = gen_entity
                
            elif geom_data.type() == sdf.GeometryType.PLANE:
                print(f"Adding Plane for link {name}: pos={pos}")
                gen_entity = self.scene.add_entity(gs.morphs.Plane(pos=pos))
                self.entity_map[parent_link_entity] = gen_entity
                    
        # If no entities added, add a default ground plane to avoid crash
        if not self.entity_map:
            print("No collision geometry found in ECM, adding default ground plane.")
            self.scene.add_entity(gs.morphs.Plane())
            
        self.scene.build()
        
        # 4. Cache relationships internally
        # Group simulated links by model
        model_simulated_links = {}
        for parent_link_entity in self.entity_map.keys():
            parent_comp = _ecm.component(parent_link_entity, components.ParentEntity)
            if parent_comp:
                model_entity = parent_comp
                if model_entity not in model_simulated_links:
                    model_simulated_links[model_entity] = []
                model_simulated_links[model_entity].append(parent_link_entity)
                
        # For each model, find its reference link
        model_ref_links = {}
        for model_entity, links in model_simulated_links.items():
            ref_link = None
            for link_entity in links:
                if _ecm.component(link_entity, components.CanonicalLink):
                    ref_link = link_entity
                    break
            
            if not ref_link:
                ref_link = links[0] # Fallback
                
            model_ref_links[model_entity] = ref_link
            
        # Cache GenesisLinkCache internally to simulated links
        for link_entity in self.entity_map.keys():
            parent_comp = _ecm.component(link_entity, components.ParentEntity)
            model_entity = parent_comp if parent_comp else None
            ref_link_entity = model_ref_links.get(model_entity)
            
            self.genesis_caches[link_entity] = {
                'ref_link_entity': ref_link_entity,
                'model_entity': model_entity
            }
            print(f"Attached GenesisLinkCache internally to link {link_entity}: ref_link={ref_link_entity}, model={model_entity}")

    def update(self, _info, _ecm):
        t_t0 = time.perf_counter()
        if _info.paused:
            return
            
        t_s0 = time.perf_counter()
        self.scene.step(update_visualizer=False, refresh_visualizer=False)
        self.profile_data['solver'] += time.perf_counter() - t_s0

        # Rate limiting
        should_update_ecm = True
        if self.ecm_update_rate > 0.0:
            current_time = time.monotonic()
            if current_time - self.last_component_update_time < 1.0 / self.ecm_update_rate:
                should_update_ecm = False
            else:
                self.last_component_update_time = current_time
                
        if not should_update_ecm:
            return
            
        # Group simulated links by model
        model_simulated_links = {}
        for link_entity, cache in self.genesis_caches.items():
            model_entity = cache['model_entity']
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
                
            ref_link_entity = self.genesis_caches[links[0]]['ref_link_entity']
            ref_gen_entity = self.entity_map.get(ref_link_entity)
            if not ref_gen_entity:
                continue
                
            ref_pos = to_numpy(ref_gen_entity.get_pos())
            quat = to_numpy(ref_gen_entity.get_quat())
            ref_q_C = gz.math.Quaterniond(quat[0], quat[1], quat[2], quat[3])
            
            # Canonical link relative pose in model frame
            ref_pose_in_model = link_pose_map.get(ref_link_entity)
            if not ref_pose_in_model:
                continue
            p_MC = ref_pose_in_model.pos()
            q_MC = ref_pose_in_model.rot()
            
            # Model orientation: q_M = q_C * q_MC^-1
            q_M = ref_q_C * q_MC.inverse()
            
            # Model world position: p_M = p_C - q_M * p_MC
            p_MC_world = q_M * gz.math.Vector3d(p_MC.x(), p_MC.y(), p_MC.z())
            p_M = ref_pos - np.array([p_MC_world.x(), p_MC_world.y(), p_MC_world.z()])
            
            # Update model pose in Gazebo
            model_pose_comp.pos().set(p_M[0], p_M[1], p_M[2])
            model_pose_comp.rot().set(q_M.w(), q_M.x(), q_M.y(), q_M.z())
            _ecm.set_changed(model_entity, components.Pose.type_id, ComponentState.PeriodicChange)
            
            # Update all other non-canonical links' relative poses
            for l in links:
                if l == ref_link_entity:
                    continue
                gen_entity = self.entity_map.get(l)
                if not gen_entity:
                    continue
                link_pos = to_numpy(gen_entity.get_pos())
                link_quat = to_numpy(gen_entity.get_quat())
                q_L = gz.math.Quaterniond(link_quat[0], link_quat[1], link_quat[2], link_quat[3])
                
                rel_pos_world = gz.math.Vector3d(link_pos[0] - p_M[0], link_pos[1] - p_M[1], link_pos[2] - p_M[2])
                rel_pos_model = q_M.inverse() * rel_pos_world
                
                rel_rot_model = q_M.inverse() * q_L
                
                link_pose_comp = link_pose_map.get(l)
                if link_pose_comp:
                    link_pose_comp.pos().set(rel_pos_model.x(), rel_pos_model.y(), rel_pos_model.z())
                    link_pose_comp.rot().set(rel_rot_model.w(), rel_rot_model.x(), rel_rot_model.y(), rel_rot_model.z())
                    _ecm.set_changed(l, components.Pose.type_id, ComponentState.PeriodicChange)
        self.profile_data['total'] += time.perf_counter() - t_t0
        self.profile_data['count'] += 1
        # self._check_profile()

        return

    def _check_profile(self):
        if self.profile_data['count'] >= 100:
            count = self.profile_data['count']
            print(f"--- Profile (avg over {count} steps) ---")
            print(f"  Solver: {self.profile_data['solver']/count*1000:.3f} ms/step")
            print(f"  Total: {self.profile_data['total']/count*1000:.3f} ms/step")
            # Reset
            self.profile_data['solver'] = 0.0
            self.profile_data['total'] = 0.0
            self.profile_data['count'] = 0
def get_system():
    return GenesisPhysics()
