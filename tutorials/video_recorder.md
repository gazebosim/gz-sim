\page videorecorder Video Recorder

## Using the video recorder plugin

Ignition Gazebo offers an video recorder tool for recording video from the 3D
Scene in the rendering window. The tool is available as a GUI plugin. To open
this plugin, first launch Ign Gazebo and select the ellipsis menu on top right
(3 dots menu), and scroll down to find the `Video Recorder` option. Click on
the plugin to open the Video Recorder tool.

Inside this tool, you should see a single button with a video recorder icon.
Clicking on the button gives you the video format options that are available.

![Video Recorder](https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/983d50937cbcaa75c790515b2ec5797fe82f1188/tutorials/files/video_recorder/video_recorder.png)

Once an option is selected, recording starts immediately as now indicated by
the flashing video recorder icon. At anytime that you wish to stop recording,
click on flashing icon and select `Stop`. A file dialog window should pop up
and let you select the path to save the recorded file in.


## Video recorder configurations

A few video recorder configurations can be specified through the gui.config
file, see the [GUI Configuration](gui_config.html) tutorial for more
information about where to locate the file and how to load different
configurations. For this tutorial, we will assume that you will be using the
default gui.config located in `$HOME/.ignition/gazebo/gui.config`.

Recall that videos are recorded from the 3D Scene, we will add the video
configuration options to the 3D scene plugin. Here is example of the
Scene 3D plugin with custom video recording settings:

```xml
<plugin filename="GzScene3D" name="3D View">
  <ignition-gui>
    <title>3D View</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="string" key="state">docked</property>
  </ignition-gui>

  <engine>ogre2</engine>
  <scene>scene</scene>
  <ambient_light>0.4 0.4 0.4</ambient_light>
  <background_color>0.8 0.8 0.8</background_color>
  <camera_pose>6 0 6 0 0.5 3.14</camera_pose>

  <record_video>
    <use_sim_time>true</use_sim_time>
    <lockstep>true</lockstep>
    <bitrate>8000000</bitrate>
  </record_video>

</plugin>
```

Options are:

* `<use_sim_time>`: Values are `[true|false]`. Record videos based on sim time,
i.e. each frame encoded into the video will be timestamped using sim time.
For example, if a complex simulation was running at half of real time, and
`<use_sim_time>` is set to true, video playback should ignore delays due
to low Real Time Factor (RTF) and plays back simulation as if RTF was 1.0.
By default, the value is `false`, which means the videos are recorded in real
time.

* `<lockstep>`: Values are `[true|false]`. Lockstep simulation for video
recording. This forces the GUI to pause and only process a new state update
from the server until the video recorder finishes encoding the current frame.
This ensures that the video recorder does not miss any updates / frames in the
resulting video. This configuration make more sense when used with
`<use_sim_time>` set to `true`, in which case it produces smooth videos
with exact timing, i.e. if you record simulation for 1 minute sim time,
the resulting video should be also 1 min long (+/- 1 second due to encoder
settings). Defaults to `false`. Note: the server publishes states at 60Hz
and the video recorder records at 25 FPS so it also makes sense to update the
Scene Broadcaster system to only publish states at 25Hz. You can do this by
going to the world sdf file, locate the
`ignition::gazebo::systems::SceneBroadcaster` system, and set the
`<state_hertz>` parameter:

    ```xml
        <plugin filename='ignition-gazebo-scene-broadcaster-system'
                name='ignition::gazebo::systems::SceneBroadcaster'>
          <state_hertz>25</state_hertz>
        </plugin>
    ```


* `<bitrate>`: Video encoding bitrate in bps. This affects the quality of the
generated video. The default bitrate is 2Mbps.


