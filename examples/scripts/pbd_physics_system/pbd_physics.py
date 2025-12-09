
from gz.sim import Model, World, world_entity, Link
from gz.sim import components, ComponentState
from gz.math import Vector3d, Pose3d
from enum import Enum
import sdformat as sdf

# PBD Constants
GRAVITY = Vector3d(0, 0, -9.81)
PBD_ITERATIONS = 100

class ShapeType(Enum):
    SPHERE = 1
    BOX = 2
    CYLINDER = 3

class PBDObject:
    def __init__(self, entity, initial_pose, shape_type, dimensions, inv_mass):
        self.entity = entity  # gz.sim.Model object
        self.initial_pose = initial_pose
        self.position = initial_pose.pos()
        self.predicted_position = initial_pose.pos()
        self.velocity = Vector3d(0, 0, 0)
        self.shape_type = shape_type
        self.dimensions = dimensions
        self.inv_mass = inv_mass

class PBDPhysics:
    def __init__(self):
        self.pbd_objects = []

    def configure(self, _entity, _sdf, _ecm, _event_mgr):
        print("PBDPhysics configure method called")
        
        world_e = world_entity(_ecm)
        w = World(world_e)
        for model_entity in w.models(_ecm):
            model = Model(model_entity)
            if model.valid(_ecm):
                is_static = model.static(_ecm)
                inv_mass = 0.0 if is_static else 1.0 # Assuming mass of 1.0 for now

                shape_type = None
                dimensions = {}

                # Find the geometry of the first collision of the canonical link
                canonical_link = Link(model.canonical_link(_ecm))
                collisions = canonical_link.collisions(_ecm)
                if collisions:
                    collision_entity = collisions[0]
                    base_comp = _ecm.component(collision_entity, components.Geometry)
                    print(collision_entity, base_comp, base_comp.data())
                    if base_comp and base_comp.data():
                        geom_data = base_comp.data()
                        if geom_data.type() == sdf.GeometryType.SPHERE:
                            shape_type = ShapeType.SPHERE
                            dimensions['radius'] = geom_data.sphere_shape().radius()
                        elif geom_data.type() == sdf.GeometryType.BOX:
                            shape_type = ShapeType.BOX
                            dimensions['size'] = geom_data.box_shape().size()
                        elif geom_data.type() == sdf.GeometryType.CYLINDER:
                            shape_type = ShapeType.CYLINDER
                            dimensions['radius'] = geom_data.cylinder_shape().radius()
                            dimensions['length'] = geom_data.cylinder_shape().length()

                if not shape_type:
                    print(f"Model {model.name(_ecm)} has no supported collision geometry, skipping.")
                    continue

                initial_pose = canonical_link.world_pose(_ecm)
                if initial_pose:
                    pbd_obj = PBDObject(model, initial_pose, shape_type, dimensions, inv_mass)
                    self.pbd_objects.append(pbd_obj)
                    print(f"Found model: {model.name(_ecm)} with shape {shape_type.name}, static: {is_static}")
                else:
                    print(f"Model {model.name(_ecm)} has no world pose component, skipping.")

    def resolve_sphere_sphere(self, obj1, obj2):
        delta_pos = obj1.predicted_position - obj2.predicted_position
        dist = delta_pos.length()
        min_dist = obj1.dimensions['radius'] + obj2.dimensions['radius']

        if dist < min_dist and dist > 1e-6:
            penetration = min_dist - dist
            normal = delta_pos / dist
            
            # mass-based correction
            total_inv_mass = obj1.inv_mass + obj2.inv_mass
            if total_inv_mass == 0:
                return

            correction_vector = normal * penetration / total_inv_mass
            obj1.predicted_position += correction_vector * obj1.inv_mass
            obj2.predicted_position -= correction_vector * obj2.inv_mass

    def resolve_sphere_box(self, obj_sphere, obj_box):
        # TODO: Implement sphere-box collision
        pass

    def resolve_box_box(self, obj1, obj2):
        # TODO: Implement box-box collision
        pass

    def update(self, _info, _ecm):
        dt = _info.dt.total_seconds()
        if dt == 0:
            return

        # Phase 1: External Forces (Gravity)
        for pbd_obj in self.pbd_objects:
            if pbd_obj.inv_mass == 0:
                continue
            pbd_obj.velocity += GRAVITY * dt
            pbd_obj.predicted_position = pbd_obj.position + pbd_obj.velocity * dt

        # Phase 2: Constraint Resolution
        for _ in range(PBD_ITERATIONS):
            # Ground Collision
            for pbd_obj in self.pbd_objects:
                if pbd_obj.shape_type == ShapeType.SPHERE:
                    radius = pbd_obj.dimensions['radius']
                    if pbd_obj.predicted_position[2] < radius:
                        pbd_obj.predicted_position[2] = radius
                # TODO: Add ground collision for other shapes

            # Object-Object Collision
            for i in range(len(self.pbd_objects)):
                for j in range(i + 1, len(self.pbd_objects)):
                    obj1 = self.pbd_objects[i]
                    obj2 = self.pbd_objects[j]

                    if obj1.inv_mass == 0 and obj2.inv_mass == 0:
                        continue
                    
                    if obj1.shape_type == ShapeType.SPHERE and obj2.shape_type == ShapeType.SPHERE:
                        self.resolve_sphere_sphere(obj1, obj2)
                    elif obj1.shape_type == ShapeType.SPHERE and obj2.shape_type == ShapeType.BOX:
                        self.resolve_sphere_box(obj1, obj2)
                    elif obj1.shape_type == ShapeType.BOX and obj2.shape_type == ShapeType.SPHERE:
                        self.resolve_sphere_box(obj2, obj1)
                    elif obj1.shape_type == ShapeType.BOX and obj2.shape_type == ShapeType.BOX:
                        self.resolve_box_box(obj1, obj2)


        # Phase 3: Update Positions and Velocities
        for pbd_obj in self.pbd_objects:
            if pbd_obj.inv_mass == 0:
                continue
            pbd_obj.velocity = (pbd_obj.predicted_position - pbd_obj.position) / dt
            pbd_obj.position = pbd_obj.predicted_position

        # Phase 4: Apply to gz-sim
        for pbd_obj in self.pbd_objects:
            if pbd_obj.inv_mass == 0:
                continue
            new_pose = Pose3d(pbd_obj.position, pbd_obj.initial_pose.rot()) # Keep original rotation
            world_pose_comp = _ecm.component(pbd_obj.entity.entity(), components.Pose)
            print("new pose: ", new_pose)
            world_pose_comp.set_data(new_pose)
            _ecm.set_changed(pbd_obj.entity.entity(), world_pose_comp.typeId, ComponentState.PeriodicChange)

def get_system():
    return PBDPhysics()
