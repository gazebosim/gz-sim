\page log Logging

The logging system allows for recording and playing back world state
information. Currently, only link poses are supported. In the future, other
state information will be supported.

## Record from command line

Run the example world with `--record` flag. This records data to a default path, i.e. `~/.ignition/gazebo/log/<timestamp>`:

`ign gazebo -v 4 -r --record pose_publisher.sdf`

A custom path can be specified for recorded files through the `--record-path` flag. When `--record-path` is specified, `--record` does not need to be separately specified:

`ign gazebo -v 4 -r --record-path ./foo pose_publisher.sdf `

Other options for recording:

`--log-overwrite`: If the record path already exists, overwrite it.

`--log-compress`: Compress the recorded file.

## Record by specifying plugin in SDF

Recording can be specified in the SDF, under `<world>` tag:

```{.xml}
<world name="default">
    ...
    <plugin
      filename="libignition-gazebo-log-system.so"
      name="ignition::gazebo::systems::LogRecord">
      <!-- Optional, directories to write recorded files. If unspecified,
             will record to default. Will be deprecated in future versions
             of Ignition. Recommended way is to specify from command line. -->
      <path>/tmp/log</path>
    </plugin>
    ...
</world>
```

If `<path>` is not specified, recorded files will be placed in the default
path (`~/.ignition/gazebo/log/<timestamp>`).

Currently, it is enforced that only one recording instance is allowed to
start during a Gazebo run.

\note If both a record plugin and a record command line flag are specified, e.g. `ign gazebo -v 4 -r --record log_record_shapes.sdf`, the command line flag will be ignored, and recorded files will be placed in the path specified in the plugin SDF (or default if none specified).

## Playback from command line

Playback can be triggered by `--playback` command line flag. `<path>` is the
directory specified to record:

`ign gazebo -r -v 4 --playback <path>`


## Playback by specifying plugin in SDF

Alternatively, playback can be specified in an SDF file. See example file
`examples/worlds/log_playback.sdf`:

```{.xml}
<world name="default">
    ...
    <plugin
      filename='libignition-gazebo-log-system.so'
      name='ignition::gazebo::systems::LogPlayback'>
      <path>/tmp/log</path>
    </plugin>
    ...
</world>
```

\note The physics plugin should not be specified in the SDF. If specified,
it will be automatically removed so that physics does not clash with the
recorded poses.

\note If both a world file `<file>` and `--playback <path>` are
specified, an error is printed. This is not allowed, because the world file
may be a very different world from the one that was recorded.

## Known issues

* When using command-line playback there is currently a small caveat.
In the case that the recorded file uses `ogre2`, the playback appears
brighter, because the default SDF that is loaded by Server.cc for playback
uses `ogre`.

* Currently, specifying record and playback at the same time is not allowed.
We may support this in the future, to support cropping a recording or
changing the encoding.
