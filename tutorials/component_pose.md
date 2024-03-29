\page posecomponent Case Study: Using the Pose Component

We will show how to use the gz::sim::components::Pose component in a system.

An example usage of the component can be found in the
gz::sim::systems::OdometryPublisher system
([source code](https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems/odometry_publisher)),
which reads the pose component of a model through the Model entity, uses the
pose for some calculations, and then publishes the result as a message.

More usage can be found in the
[integration test](https://github.com/gazebosim/gz-sim/blob/gz-sim8/test/integration/odometry_publisher.cc)
for the system, with test worlds `odometry*.sdf`
[here](https://github.com/gazebosim/gz-sim/tree/main/test/worlds).

### Objects of interest

- gz::sim::components::Pose: A component containing pose information
- gz::math::Pose3d: The actual data underlying a pose component
- gz::sim::systems::OdometryPublisher: A system that reads the pose component
  of a model
- gz::sim::Model: The type underlying a model entity (gz::sim::Entity)

### Find the model entity

First, we will need access to an entity, the \ref gz::sim::Model entity in this
case.
`OdometryPublisher` happens to be a system meant to be specified under `<model>`
in the SDF, so at the time `Configure()` is called, it has access to a model
entity from which we can extract a \ref gz::sim::Model:

\snippet src/systems/odometry_publisher/OdometryPublisher.cc modelDeclaration
\snippet src/systems/odometry_publisher/OdometryPublisher.cc Configure

### Read the pose component

Once we have the handle to an entity, we can access components associated with
it.
A component may have been created at the time the world is loaded, or you may
create a component at runtime if it does not exist yet.

In this case, we use the model entity found above to access its pose component,
which is created by default on every model entity.

In `PostUpdate()`, which happens after physics has updated, we can get the
world pose of a model through gz::sim::worldPose, by passing in the model
entity and the entity component manager.

\snippet src/systems/odometry_publisher/OdometryPublisher.cc worldPose

It returns the raw data to us in the gz::math::Pose3d type, which is also the
data type underlying a pose component.
We can perform calculations on the gz::math::Pose3d type, not the
gz::sim::components::Pose type, which is just a wrapper.

### Use the pose component

Now we can use the pose data as we like.

Here, we manipulate the pose and package the result into a gz::msgs::Odometry
message to be published:

\snippet src/systems/odometry_publisher/OdometryPublisher.cc declarePoseMsg

\snippet src/systems/odometry_publisher/OdometryPublisher.cc setPoseMsg

See the source code for setting other fields of the message, such as twist and
the header.

The message is then published:

\snippet src/systems/odometry_publisher/OdometryPublisher.cc publishMsg

where `odomPub` is defined in `Configure()`:

\snippet src/systems/odometry_publisher/OdometryPublisher.cc definePub

Outside the scope of this tutorial, the odometry publisher system also
calculates the covariance and publishes a pose vector on a TF topic.
See the source code to learn more.
