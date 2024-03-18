\page videorecorder Video Recorder

## Using the video recorder plugin

Gazebo offers a video recorder tool for recording videos from the 3D
scene. The recorder tool is available as a GUI plugin. To open this plugin,
first launch Gazebo and select the ellipsis menu on top right
(3 dots menu), and scroll down to find the `Video Recorder` option. Click on the
plugin to open the Video Recorder tool. Alternatively, launch the demo world in
Gazebo that already has this plugin included in the GUI.

```
gz sim -v 4 video_record_dbl_pendulum.sdf
```

In this demo world, you should see a video recorder icon positioned on the top.
left area of the window along with other buttons. Clicking on the video
recorder button gives you the video format options that are available.

@image html files/video_recorder/video_recorder.png

Once an option is selected, recording starts immediately as indicated by
a flashing video recorder icon. At anytime that you wish to stop recording,
click on the flashing icon and select `Stop`. A file dialog window should pop up
and let you select the path to save the recorded video in.

Playback the video you just saved and you should notice that the resolution
of the video is based on the size of your 3D scene window. So if you wish
to record the video in a different size, make sure to configure the GUI
window prior to recording.

@image html files/video_recorder/video_recorder.gif


## Video recorder configurations

A few video recorder parameters can be specified using GUI configurations, see
the [GUI Configuration](gui_config.html) tutorial for more information.
If you launched Gazebo with the
`video_record_dbl_pendulum.sdf` demo world, the GUI configurations are embedded
in the world SDF file so you will need to download a copy of the
[sdf file](https://raw.githubusercontent.com/gazebosim/gz-sim/main/examples/worlds/video_record_dbl_pendulum.sdf).
and modify the GUI configuration in that file. On the other hand, if you
launched Gazebo with a world file that does not have GUI
configurations, you will need to specify the settings in
`$HOME/.gz/sim/<#>/gui.config`.

Here is an example of the video recorder plugin's settings:

```xml
<plugin filename="VideoRecorder" name="VideoRecorder">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="x" type="double">300</property>
    <property key="y" type="double">50</property>
    <property key="width" type="double">50</property>
    <property key="height" type="double">50</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
    <property key="cardBackground" type="string">#777777</property>
  </gz-gui>

  <record_video>
    <use_sim_time>true</use_sim_time>
    <lockstep>true</lockstep>
    <bitrate>4000000</bitrate>
  </record_video>
</plugin>
```

Options are:

* **use_sim_time**: Values are `[true|false]`. Record videos based on sim time,
i.e. each frame encoded into the video will be timestamped using sim time.
For example, if a complex simulation was running at half of real time speed, and
`<use_sim_time>` is set to true, video playback should ignore delays due
to low Real Time Factor (RTF) and plays back video as if RTF was 1.0.
By default, the value is `false`, which means the videos are recorded based
on real time.

* **lockstep**: Values are `[true|false]`. Lockstep simulation for video
recording. This forces the GUI to pause and only process a new state update
from the server until the video recorder finishes encoding the current frame.
This ensures that the video recorder does not miss any updates / frames in the
resulting video. This configuration makes more sense when used with
`<use_sim_time>` set to `true`, in which case it produces smooth videos
with exact timing, i.e. if you record simulation for 1 minute sim time,
the resulting video should be also 1 minute long (+/- 1 second due to encoder
settings). Defaults to `false`. Note: the server publishes states at 60Hz
and the video recorder records at 25 FPS so it also makes sense to update the
Scene Broadcaster system to only publish states at 25Hz. You can do this by
going to the world SDF file, locate the
`gz::sim::systems::SceneBroadcaster` system, and set the
`<state_hertz>` parameter:

```xml
        <plugin filename='gz-sim-scene-broadcaster-system'
                name='gz::sim::systems::SceneBroadcaster'>
          <state_hertz>25</state_hertz>
        </plugin>
```

* **bitrate**: Video encoding bitrate in bps. This affects the quality of the
generated video. The default bitrate is 2Mbps.

## Hardware-accelerated encoding

Since Gazebo Common 3.10.2, there is support for utilizing the power of GPUs
to speed up the video encoding process. See the
[Hardware-accelerated Video Encoding tutorial](https://gazebosim.org/api/common/5/hw-encoding.html)
for more details.
