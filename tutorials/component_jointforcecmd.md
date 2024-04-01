\page jointforcecmdcomponent Case Study: Using the JointForceCmd Component

We will show how to use one of the components,
\ref gz::sim::components::JointForceCmd, in a system.
This component allows us to set the force command on a joint.

Programmatic usage of this component can be found in the source code for
systems and integration tests, such as the
[joint integration test](https://github.com/gazebosim/gz-sim/blob/gz-sim8/test/integration/joint.cc),
the \ref gz::sim::systems::ApplyJointForce system
([source code](https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems/apply_joint_force)),
and others.

The corresponding world SDF is [`apply_joint_force.sdf`](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/apply_joint_force.sdf), which you can look at in Gazebo:

```bash
gz sim apply_joint_force.sdf
```

We will walk through the relevant lines of source code in `ApplyJointForce`
that interact with `JointForceCmd`.

### Find the entity of interest

First, we will need access to an entity, the \ref gz::sim::Joint entity in this
case. It is declared as a member variable:

\snippet src/systems/apply_joint_force/ApplyJointForce.cc jointEntityDeclaration

An entity may have been created at the time the world is loaded, or you may
create an entity at runtime if it does not exist yet.
For joints, most likely they were defined in the SDF file that specifies the
world, and all we have to do at runtime is to look for the joint by its name.

`ApplyJointForce` happens to be a system meant to be specified under `<model>`
in the SDF, so at the time `Configure()` is called, it has access to a model
entity from which we can extract a \ref gz::sim::Model:

\snippet src/systems/apply_joint_force/ApplyJointForce.cc modelDeclaration
\snippet src/systems/apply_joint_force/ApplyJointForce.cc Configure

Using the Model object, we can find the joint by its name, when `PreUpdate()`
is called.
That gives us a Joint entity:

\snippet src/systems/apply_joint_force/ApplyJointForce.cc findJoint

### Modify the component

Once we have the handle to an entity, we can modify components associated with
it.
A component may have been created at the time the world is loaded, or you may
create a component at runtime if it does not exist yet.

In this case, we use the joint entity found above to look for and modify its
`JointForceCmd` component.
This will apply a force command to the joint.

In `PreUpdate()`, look for the component:

\snippet src/systems/apply_joint_force/ApplyJointForce.cc jointForceComponent

Create it if it does not exist yet, and modify it:

\snippet src/systems/apply_joint_force/ApplyJointForce.cc modifyComponent

where the scalar joint force command is declared as a member variable:

\snippet src/systems/apply_joint_force/ApplyJointForce.cc forceDeclaration

and a callback function allows the user to specify a force on a topic:

\snippet src/systems/apply_joint_force/ApplyJointForce.cc cmdTopic
\snippet src/systems/apply_joint_force/ApplyJointForce.cc cmdSub
\snippet src/systems/apply_joint_force/ApplyJointForce.cc setForce

You can test this by issuing a force command to the topic:

```bash
gz topic -t /model/joint_force_example/joint/j1/cmd_force \
  -m gz.msgs.Double -p 'data: 1.0'
```
This should move the model that the joint is attached to.
