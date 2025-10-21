\page websocket_server Websocket server for gzweb

The websocket server system starts up a websocket server for the web-based
Gazebo visualization tool, [gzweb](https://github.com/gazebo-web/gzweb).

## Usage

The websocket server system can be included in an world SDF file inside the
`<world>` SDF element like other systems.

A simple example is shown below:

```xml
    <plugin
        filename="gz-sim-websocket-server-system"
        name="gz::sim::systems::WebsocketServer">
      <port>9002</port>
      <publication_hz>30</publication_hz>
      <max_connections>-1</max_connections>
    </plugin>
```

In this example, the websocket server will be launched on port 9002, the data
published to gzweb are throttled at 30Hz, and the server has no limit on the
maximum number of client connections. Authorization and SSL settings can also
be configured.

See more information on the plugin parameters and how to launch gzweb in the
[websocket server demo](https://github.com/gazebosim/gz-sim/blob/gz-sim10/examples/scripts/websocket_server)

@image html files/websocket_server/fuel_visualization_gzweb.png

### Launch websocket server separately from Gazebo

It is possible to launch the websocket server separately while Gazebo is
running. This can be done by making use of gz-sim's
`/world/<world_name>/system/add` service. For example:

1. Launch `shapes.sdf` world

    ```bash
    gz sim -v 4 shapes.sdf -s
    ```

1. In another terminal, make a request to the gz-sim service to add the
websocket server system

    ```bash
    gz service -s /world/shapes/entity/system/add --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 2000 --req 'plugins: {name: "gz::sim::systems::WebsocketServer", filename: "gz-sim-websocket-server-system", innerxml: "<port>9002</port><publication_hz>30</publication_hz><max_connections>-1</max_connections>"}'
    ```

    Check the console output in the terminal that you used to launch gz sim
    and verify that the websocket server is started.

## Migrating from gz-launch

The websocket server implementation was migrated from gz-launch as gz-launch
is deprecated in Gazebo Jetty. It is converted to a gz-sim system and offers
the same functionality as the one in gz-launch. All websocket server plugin
parameters that were available in gz-launch are supported in the gz-sim system.

In gz-launch, the websocket server exists in the form of a "gz-launch plugin",
which can be started from a launch file, see the example
[websocket.gzlaunch](https://github.com/gazebosim/gz-launch/blob/gz-sim10/examples/websocket.gzlaunch)
file.

Specifically, the gz-launch plugin has the following syntax:

```xml
  <plugin name="gz::launch::WebsocketServer"
          filename="gz-launch-websocket-server">
    ...
  </plugin>
```

Now that in gz-sim, the websocket server is a gz-sim system, simply take the
contents of the gz-launch plugin, add them to your SDF world file, and change
the plugin name and filename to:

```xml
    <plugin
        filename="gz-sim-websocket-server-system"
        name="gz::sim::systems::WebsocketServer">
      ...
    </plugin>
```

Launch the SDF world file as using by running `gz sim <path_to_sdf>`, the
websocket server should be launched together with Gazebo.
