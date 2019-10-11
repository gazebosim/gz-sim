# Log Video Recorder

This demo shows how to record videos from a log playback. The video is recorded
from the view of the GUI camera which is set to follow entities in the world.
One video will be created per entity.

How are the entities chosen? Currently models that are within a hardcoded region
in the world will be recorded. A TODO item is to let users choose what entities
to record videos for.

## Running the script

The demo can be started by running the script:

        bash record_one_run.bash [path_to_log]

e.g.

        bash record_one_run.bash /tmp/ign/logs

Once the script is run, ign-gazebo window should pop-up and the log playback
should automatically start. The GUI camera will follow the first entity found
and the video recorder will be started. When the log playback ends, the video
is saved to a temporary directory (`tmp_recording`). The log playback rewinds
and the process is repeated for next entity until there are no more entities
left. Once all videos are done, ign-gazebo is killed and the videos are moved
to a timestamped directory where the `record_one_run.bash` is in.

## Changing camera follow behavior

The camera follow behavior can be configured by setting the `<camera_follow>`
parameters in the GzScene3d GUI plugin in `log_video_recorder.sdf`, i.e.

        <camera_follow>
          <p_gain>0.05</p_gain>
          <world_frame>true</world_frame>
          <offset>-1.0 0 2.5</offset>
        </camera_follow>

