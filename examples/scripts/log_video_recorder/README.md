# Log Video Recorder

This demo shows how to record videos from a log playback using the
`LogVideoRecorder` system. The video is recorded from the view of the GUI camera
which is set to follow entities in the world. One video will be created per
entity. The entites can be specified by its name and by region (axis-aligned
box) where the entities are located at the start of the log.

## Running the demo

Run the `log_record_shapes.sdf` world to generate a state log of the simulation:

    gz sim log_record_shapes.sdf

Press Play and let the simulation run for 10 seconds of sim time. Stop and exit.
You should now see a `state.tlog` generated in `/tmp/log` directory. Now, let's
record videos from playing back this log file.

Start the log playback recorder by running the script:

        bash record_one_run.bash [path_to_log]

e.g.

        bash record_one_run.bash /tmp/log

Once the script is run, gz-sim window should pop-up and log playback
should automatically start. The GUI camera will follow the first entity found
and the video recorder will be started. When the log playback ends, the video
is saved to the current working directory. The log playback rewinds and the
process is repeated for next entity until there are no more entities left. Once
all videos are done, gz-sim is killed and the videos are moved to a
timestamped directory where the `record_one_run.bash` is in.

## Changing camera follow behavior

> This feature hasn't been ported to Fortress yet, see
> https://github.com/gazebosim/gz-gui/issues/298

## Troubleshooting

1. The world / models are not being loaded on log playback?
    A: Make sure you have all the fuel models downloaded in ~/.gz/fuel cache
directory

1. I see `tmp_record` dir, `tmp_recording.mp4` and other left over mp4 files
in the directory where I ran `record_one_run.bash script.
    A: The playback and recording process is probably interrupted and the
script did not have the chance to clean up some of these intermediate files

1. The video is missing the beginning of the playback.
    A: The log playback recorder system has to wait until an entity (that you want
to record a video for) appears before doing any recordings.
