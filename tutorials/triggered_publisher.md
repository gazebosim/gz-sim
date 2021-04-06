\page triggeredpublisher Triggered Publisher

The `TriggeredPublisher` system publishes a user specified message on an output
topic in response to an input message that matches user specified criteria. The
system works by checking the input against a set of Matchers. Matchers
contain string representations of protobuf messages which are compared for
equality or containment with the input message. Matchers can match the whole
input message or only a specific field inside the message.

This tutorial describes how the Triggered Publisher system can be used to
cause a box to fall from its initial position by detaching a detachable joint
in response to the motion of a vehicle. The tutorial also covers how Triggered
Publisher systems can be chained together by showing how the falling of the box
can trigger another box to fall. The finished world SDFormat file for this
tutorial can be found in
[examples/worlds/triggered_publisher.sdf](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo2/examples/worlds/triggered_publisher.sdf)

We will use the differential drive vehicle from
[examples/worlds/diff_drive.sdf](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo2/examples/worlds/diff_drive.sdf),
but modify the input topic of the `DiffDrive` system to `cmd_vel`. A snippet of
the change to the `DiffDrive` system is shown below:

```xml
<model name='vehicle_blue'>
  ...

  <plugin
    filename="ignition-gazebo-diff-drive-system"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>front_left_wheel_joint</left_joint>
    <left_joint>rear_left_wheel_joint</left_joint>
    <right_joint>front_right_wheel_joint</right_joint>
    <right_joint>rear_right_wheel_joint</right_joint>
    <wheel_separation>1.25</wheel_separation>
    <wheel_radius>0.3</wheel_radius>
    <topic>cmd_vel</topic>
  </plugin>
</model>

```

The first `TriggeredPublisher` we create will demonstrate how we can send
a predetermined `Twist` message to the `DiffDrive` vehicle in response to
a "start" message from the user:

```xml
<plugin filename="ignition-gazebo-triggered-publisher-system"
        name="ignition::gazebo::systems::TriggeredPublisher">
  <input type="ignition.msgs.Empty" topic="/start"/>
  <output type="ignition.msgs.Twist" topic="/cmd_vel">
      linear: {x: 3}
  </output>
</plugin>
```

The `<input>` tag sets up the `TriggeredPublisher` to subscribe to the topic
`/start` with a message type of `ignition.msgs.Empty`. The `<output>` tag
specifies the topic of the output and the actual data to be published. The data
is expressed in the human-readable form of Google Protobuf messages. This is
the same format used by `ign topic` for publishing messages.

Since the `TriggeredPublisher` only deals with Ignition topics, it can be
anywhere a `<plugin>` tag is allowed. For this example, we will put it under
`<world>`.

Next we will create a trigger that causes a box to fall when the `DiffDrive`
vehicle crosses a contact sensor on the ground. To do this, we first create the
falling box model and call it `box1`

```xml
<model name="box1">
  <pose>3 0 8 0 0 0</pose>
  <link name="box_body">
    <visual name="v1">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <diffuse>0.8 0.2 0.2 1</diffuse>
        <specular>1.0 0 0 1</specular>
      </material>
    </visual>
    <collision name="c1">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
  </link>
</model>
```

For now, the model will only contain a single link with a `<visual>` and a
`<collision>`. Next, we create a model named "trigger" that contains the
contact sensor, the `TouchPlugin` and `DetachableJoint` systems as well as visuals
indicating where the sensor is on the ground.

```xml

<model name="trigger">
    <pose>3 0 0 0 0 0</pose>
    <static>true</static>
    <link name="body">
      <visual name="v1">
        <geometry>
          <box><size>0.1 10 0.01</size></box>
        </geometry>
      </visual>
      <collision name="c1">
        <geometry>
          <box><size>0.1 10 0.01</size></box>
        </geometry>
      </collision>
      <sensor name='sensor_contact' type='contact'>
        <contact>
          <collision>c1</collision>
        </contact>
      </sensor>
    </link>
    <plugin
      filename="ignition-gazebo-touchplugin-system"
      name="ignition::gazebo::systems::TouchPlugin">
      <target>vehicle_blue</target>
      <namespace>trigger</namespace>
      <time>0.001</time>
      <enabled>true</enabled>
    </plugin>
    <plugin filename="ignition-gazebo-detachable-joint-system"
            name="ignition::gazebo::systems::DetachableJoint">
      <parent_link>body</parent_link>
      <child_model>box1</child_model>
      <child_link>box_body</child_link>
      <topic>/box1/detach</topic>
    </plugin>
  </model>

```

\note The contact sensor needs the `Contact` system under `<world>`

```xml
    <world>
      ...
      <plugin
        filename="ignition-gazebo-contact-system"
        name="ignition::gazebo::systems::Contact">
      </plugin>
      ...
    </world>
```

The `DetachableJoint` system creates a fixed joint between the link "body" in
`trigger` and the link "box_body" in `box1`.  The model `trigger` is a static
model, hence, `box1` will remain fixed in space as long as it is attached to
`trigger`. The `DetachableJoint` system subscribes to the `/box1/detach` topic
and, on receipt of a message, will break the fixed joint and lets `box1` fall
to the ground.

When the vehicle runs over the contact sensor associated with `c1`, the
`TouchPlugin` will publish a message on `/trigger/touched`. We will use this as
our trigger to send a message to `/box1/detach`. The `TouchPlugin` publishes
only when there is contact, so we can trigger on any received message. However,
to demonstrate the use of matchers, we will only trigger when the Boolean input
message is `true`

```xml
<plugin filename="ignition-gazebo-triggered-publisher-system"
        name="ignition::gazebo::systems::TriggeredPublisher">
  <input type="ignition.msgs.Boolean" topic="/trigger/touched">
    <match>data: true</match>
  </input>
  <output type="ignition.msgs.Empty" topic="/box1/detach"/>
</plugin>
```

Finally, we will use an Altimeter sensor to detect when `box1` has fallen to
the ground to cause another box to fall. We will add the Altimeter sensor to
the link "box_body" in `box1`

```xml

<model name="box1">
  <pose>3 0 8 0 0 0</pose>
  <link name="box_body">
    ...
    <sensor name="altimeter_sensor" type="altimeter">
      <topic>/altimeter</topic>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
    </sensor>
  </link>
</model>
```

\note The Altimeter sensor needs the `Altimter` system under `<world>`
```xml
    <world>
      ...
      <plugin filename="ignition-gazebo-altimeter-system"
        name="ignition::gazebo::systems::Altimeter">
      </plugin>
      ...
    </world>
```

We will call the second falling box `box2` and it will contain the same types
of visuals and collisions as in box1.

```xml
<model name="box2">
  <pose>5 0 8 0 0 0</pose>
  ...
</model>
```

Again, we'll make use of the `DetachableJoint` system to attach `box2` to the
static model `trigger` by adding the following to `trigger`

```xml
<model name="trigger">
  ...
  <plugin filename="ignition-gazebo-detachable-joint-system"
          name="ignition::gazebo::systems::DetachableJoint">
    <parent_link>body</parent_link>
    <child_model>box2</child_model>
    <child_link>box_body</child_link>
    <topic>/box2/detach</topic>
  </plugin>
</model>
```

Similar to what we did for `box1`, we need to publish to `/box2/detach` when
our desired trigger occurs. To setup our trigger, we observe that the altimeter
publishes an `ignition.msgs.Altimeter` message that contains a
`vertical_position` field. Since we do not necessarily care about the values of
the other fields inside `ignition.msgs.Altimeter`, we will create a
`TriggeredPublisher` matcher that matches a specific field.

The value of the `vertical_position` field will be the altitude of the link
that it is associated with relative to its starting altitude. When `box1` falls
to the ground, the value of the altimeter will read about -7.5. However, since
we do not know the exact value and an exact comparison of floating point
numbers is not advised, we will set a tolerance of 0.2.

```xml
<plugin filename="ignition-gazebo-triggered-publisher-system"
        name="ignition::gazebo::systems::TriggeredPublisher">
  <input type="ignition.msgs.Altimeter" topic="/altimeter">
    <match field="vertical_position" tol="0.2">-7.5</match>
  </input>
  <output type="ignition.msgs.Empty" topic="/box2/detach"/>
</plugin>
```

We can now run the simulation and from the command line by running

```
ign gazebo -r triggered_publisher.sdf
```

and publish the start message

```
ign topic -t "/start" -m ignition.msgs.Empty -p " "
```
