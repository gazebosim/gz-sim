\page log Logging

Gazebo records two types of information to files:

* Console messages
    * From the server process only, not the GUI
    * Logged to a plain-text `server_console.log` file
    * Always recorded
* Simulation state
    * Entity poses, insertion and deletion
    * Logged to an [Gazebo Transport `state.tlog` file](https://gazebosim.org/api/transport/7.0/logging.html)
    * Recording must be enabled from the command line or the C++ API
    * Can be played back using the command line or the C++ API

## Record

### From command line

Run the example world with `--record` flag. This records data to a default
path, i.e. `~/.gz/sim/log/<timestamp>`:

`gz sim -v 4 -r --record pose_publisher.sdf`

A custom path can be specified for recorded files through the `--record-path`
flag. When `--record-path` is specified, `--record` does not need to be
separately specified:

`gz sim -v 4 -r --record-path ./foo pose_publisher.sdf `

Other options for recording:

* `--record-resources`: Models and textures can be recorded, in addition to
                        states, by specifying this flag. This implicitly
                        enables `--record` flag.

* `--log-overwrite`: If the record path already exists, overwrite it. Defaults to
                     false, in which case it's recorded to the given path with
                     a number appended (i.e. `/tmp/log(1)`, `/tmp/log(2)`...).

* `--log-compress`: Compress the recorded file.

### From C++ API

All features available through the command line are also available through
[gz::sim::ServerConfig](https://gazebosim.org/api/gazebo/7/classignition_1_1gazebo_1_1ServerConfig.html).
When instantiating a server programmatically, logging options can be passed
to the constructor, for example:

```
gz::sim::ServerConfig serverConfig;
serverConfig.SetUseLogRecord(true);
serverConfig.SetLogRecordPath("custom_path");

gz::sim::Server server(serverConfig);
```

### From plugin in SDF

Recording can be specified in the SDF, under `<world>` tag:

```{.xml}
<world name="default">
    ...
    <plugin
      filename="gz-sim-log-system"
      name="gz::sim::systems::LogRecord">
    </plugin>
    ...
</world>
```

Currently, it is enforced that only one recording instance is allowed to
start during a Gazebo run.

### Record path

The final record path will depend on a few options:

* If state recording is not enabled, only the console log is recorded to
  `~/.gz/sim/log/<timestamp>`.
* If only `--record`, all files are recorded to
  `~/.gz/sim/log/<timestamp>`.
* If `--record-path` is specified:
    * If the path doesn't exist, logs are recorded there.
    * If the path exists:
        * If no `--log-overwrite`, logs are recorded to a new path with a number
          appended, i.e. `/tmp/log(1)`, `/tmp/log(2)`...
        * If `--log-overwrite`, the directory is cleared and logs recorded to it.

## Playback

### From command line

Playback can be triggered by `--playback` command line flag. `<path>` is the
directory specified to record:

`gz sim -r -v 4 --playback <path>`

### From plugin in SDF

Playing back via the SDF tag `<path>` has been removed.
Please use the command line argument.

## Known issues

* When using command-line playback there is currently a small caveat.
In the case that the recorded file uses `ogre2`, the playback appears
brighter, because the default SDF that is loaded by Server.cc for playback
uses `ogre`.

* Currently, specifying record and playback at the same time is not allowed.
We may support this in the future, to support cropping a recording or
changing the encoding.
