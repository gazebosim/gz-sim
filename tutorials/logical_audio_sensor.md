\page logicalaudiosensor Logical Audio Sensor

This tutorial will explain how to use the `LogicalAudioSensor` system plugin in Gazebo.

The logical audio sensor plugin allows for the usage of logical audio sources and microphones in a simulation environment.
At the end of each simulation step, microphones check if audio was detected by any of the sources in the world.
The logical audio plugin does not play actual audio to a device like speakers, but rather simulates audio being played in environments to see if audio could theoretically be heard at a certain location or not.

## Setup

Let's take a look at [logical_audio_sensor_plugin.sdf](https://github.com/gazebosim/gz-sim/blob/460d2b1cfbf0addf05a1e61c05e1f7a675a83785/examples/worlds/logical_audio_sensor_plugin.sdf), which defines a simulation world with 4 models (in this case, boxes) that have an audio object attached to them.
This world attaches logical audio sources to the `red_box` and `blue_box` models, and attaches logical microphones to the `green_box` and `yellow_box` models.

Let's take a look at the SDF relevant to the source for `red_box` to understand how to define a logical audio source in SDF:

```xml
      <plugin filename="gz-sim-logicalaudiosensorplugin-system" name="gz::sim::systems::LogicalAudioSensorPlugin">
        <source>
          <id>1</id>
          <pose>.5 0 0 0 0 0</pose>
          <attenuation_function>linear</attenuation_function>
          <attenuation_shape>sphere</attenuation_shape>
          <inner_radius>1.0</inner_radius>
          <falloff_distance>5.0</falloff_distance>
          <volume_level>.8</volume_level>
          <playing>true</playing>
          <play_duration>10</play_duration>
        </source>
      </plugin>
```

As we can see, we use a `<source>` tag to define an audio source.
An explanation of all of the tags can be found in the [plugin documentation](https://github.com/gazebosim/gz-sim/blob/314477419d2aa946f384204dc99b17d9fcd963b3/src/systems/logical_audio_sensor_plugin/LogicalAudioSensorPlugin.hh#L35-L130), but there are a few important things to point out:
* `<id>` is used to identify this source when operating on it via services (services will be discussed later).
Since a model can have multiple sources and microphones attached to it, each source attached to a particular model must have a unique ID.
This means that no other sources attached to `red_box` can have an ID of 1, but sources attached to other models can have an ID of 1 (assuming that other models don't already have a source with an ID of 1 attached to it).
* The source's pose is defined relative to the model's pose it's attached to.
In this case, since the `red_box` pose is `(0, 0, 0.5, 0, 0, 0)` relative to the world, and the source pose is `(0.5, 0, 0, 0, 0, 0)` relative to the `red_box` pose, the source's pose with respect to the world is `(0.5, 0, 0.5, 0, 0, 0)`.
* `<attenuation_function>`, `<attenuation_shape>`, `<inner_radius>`, and `<falloff_distance>` are parameters that define the source's behavior as it travels through space.
More information about how these parameters behave a source's behavior can be found [here](https://docs.unrealengine.com/en-US/Engine/Audio/DistanceModelAttenuation/index.html).

One other thing to note is that the source attached to the `blue_box` model has a `<play_duration>` of `0`.
This means that this source will play for an infinite amount of simulation time, unless it is stopped manually by the user.

Let's now take a look at the SDF relevant to the microphone for `green_box` to understand how to define a logical microphone in SDF:

```xml
      <plugin filename="gz-sim-logicalaudiosensorplugin-system" name="gz::sim::systems::LogicalAudioSensorPlugin">
        <microphone>
          <id>1</id>
          <pose>0 .5 0 0 0 0</pose>
          <volume_threshold>.4</volume_threshold>
        </microphone>
      </plugin>
```

The same rules regarding `<id>` and `<pose>` for a logical audio source also apply to a logical microphone.
You can also take a look at the [microphone documentation](https://github.com/gazebosim/gz-sim/blob/314477419d2aa946f384204dc99b17d9fcd963b3/src/systems/logical_audio_sensor_plugin/LogicalAudioSensorPlugin.hh#L35-L130) for a detailed explanation of the tags embedded in the `<microphone>` tag.

## Testing Source and Microphone Behavior

Let's use the SDF file introduced above (`logical_audio_sensor_plugin.sdf`) to see how sources and microphones interact in an environment.

Start a simulation that uses this SDF file by running the following command in a terminal:

```
gz sim logical_audio_sensor_plugin.sdf
```

You should see a something like this:

@image html files/logical_audio_sensor_plugin.png

### Observing Microphone Detections

If a logical microphone can detect a source, it will publish a message to a detection topic.
With the simulator still running, open a new terminal and run the following command to see which microphone detection topics are available:

```
gz topic -l
```

You should see the following detection topics as a part of the output (the `_1` suffix is the ID assigned to the microphones in the SDF):

```
/model/green_box/sensor/mic_1/detection
/model/yellow_box/sensor/mic_1/detection
```

Let's see if the microphone attached to `yellow_box` can hear anything.
Run the following command:

```
gz topic -e -t /model/yellow_box/sensor/mic_1/detection
```

You'll notice that this command produces no output.
This means that this microphone cannot detect any of the sources that are currently playing.

Now, let's see if the microphone attached to `green_box` can hear anything.
Modify the command you just ran to look like this:

```
gz topic -e -t /model/green_box/sensor/mic_1/detection
```

You'll notice an output that looks like the following:

```
header {
  stamp {
  }
  data {
    key: "world/logical_audio_sensor/model/red_box/sensor/source_1"
  }
}
data: 0.4900980486407216
```

This means that this microphone can detect the source connected to `red_box`, and the volume level detected by the microphone was roughly `0.49` (0.0 being 0% volume, and 1.0 being 100% volume).
Now, go ahead and start simulation by pressing the "play" icon at the bottom-left of the screen, and the detection output will now have a time stamp attached to it:

```
header {
  stamp {
    sec: 8
    nsec: 925000000
  }
  data {
    key: "world/logical_audio_sensor/model/red_box/sensor/source_1"
  }
}
data: 0.4900980486407216
```

Since the source attached to `red_box` was defined in the SDF file using `<play_duration>10</play_duration>`, this means that the last message published by the `green_box` detection topic will have a time stamp of 10 seconds.

### Starting/Stopping Audio Sources

Now that we've discussed how to observe microphone detections, let's see how we can modify the state of a logical audio source.
Logical audio sources can be started/stopped manually through Gazebo services.
To see which services to call in order to start/stop a service, open a new terminal and run the following command (make sure the simulator is still running):

```
gz service -l
```

If you look through the list of available services, you should see the following audio source services (the `_1` suffix is the ID assigned to the sources in the SDF):

```
/model/blue_box/sensor/source_1/play
/model/blue_box/sensor/source_1/stop
/model/red_box/sensor/source_1/play
/model/red_box/sensor/source_1/stop
```

Let's start the source attached to `blue_box`.
This can be done by running the following command, which calls the "play" service for the source attached to `blue_box`:

```
gz service -s /model/blue_box/sensor/source_1/play --reqtype gz.msgs.Empty --reptype gz.msgs.Boolean --timeout 1000 --req 'unused: false'
```

Now, if you look back at the terminal that is displaying the output of `green_box`'s microphone detections, we can see that this microphone is detecting audio from `blue_box`'s source (we see `blue_box` as a part of the `key` field):

```
header {
  stamp {
    sec: 255
    nsec: 673000000
  }
  data {
    key: "world/logical_audio_sensor/model/blue_box/sensor/source_1"
  }
}
data: 0.6
```

You can also echo `yellow_box`'s microphone detection topic, and you should see that this microphone can detect audio from `blue_box`'s source as well.

Since the source attached to `blue_box` was configured with `<play_duration>0</play_duration>` in SDF, it won't stop playing unless we call the stop service on it.
Before we call the stop service on this source, move `blue_box` around with the transform control tool.
You'll see that once `blue_box`'s source volume falls below a microphone's detection threshold (defined in the SDF via `<volume_threshold>`), messages will stop being published to the microphone's detection topic.

Let's go ahead and call the stop service to make sure that this source will stop playing.
Move `blue_box` back towards its original position, until you see detection messages being published by the microphones attached to `green_box` and/or `yellow_box`.
Now, go ahead and stop `blue_box`'s source by running the following command:

```
gz service -s /model/blue_box/sensor/source_1/stop --reqtype gz.msgs.Empty --reptype gz.msgs.Boolean --timeout 1000 --req 'unused: false'
```

Now, if you look at the output for either microphone topic, you'll notice that no new messages are being published, which makes sense since no audio sources are currently playing.
You can also try to call the play/stop services on the source attached to the `red_box`.
Since the `red_box` has a play duration of 10 seconds, the source will automatically stop playing 10 seconds (of sim time) after starting it, unless you stop it earlier by calling the stop service.
One other important thing to note is that calling the play service on a source that is already playing does nothing.
For example, if the source attached to `red_box` is already playing, and you call the play service on it, then it will not play for another 10 seconds.
The source will still stop at the original 10 second duration mark.
Similarly, calling the stop service on a source that is already stopped also does nothing.
