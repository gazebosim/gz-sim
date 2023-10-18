\page comppose Case Study: Using the Pose Component

We will show how to use the gz::sim::components::Pose component in a system.

For example, the gz::sim::systems::OdometryPublisher system reads the pose
component of a model through the Model entity, uses the pose for some
calculations, and then publishes the result as a message.

Objects of interest:

- gz::sim::components::Pose: A component containing pose information
- gz::math::Pose3d: The actual pose data underlying a pose component
- gz::sim::systems::OdometryPublisher: A system that reads the pose component
  of a model
- gz::sim::Model: The type underlying a model entity (gz::sim::Entity)



