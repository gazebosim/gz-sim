# Get Started

This demo shows how to launch Gazebo with a websocket server for
web visualization with [gzweb](https://github.com/gazebo-web/gzweb).

1. Launch the `websocket_server.sdf` demo world.
The demo world is no different from other Gazebo worlds except it includes the
`gz-sim-websocket-server-system`.

```bash
gz sim -v 4 -s websocket_server.sdf
```
1. View Gazebo simulation in the web

   1. Option 1: Open `index.html` in a web browser.

    ```bash
    firefox index.html
    ```
      * The `index.html` web page is a simple demo that integrates a snapshot
        version of gzweb for communicating with the local websocket server.
        It illustrates how gzweb connects to the websocket server
        the events that it listens to in order to create the scene. It is
        hardcoded to connect to the `websocket_server` world and has limited
        support for materials. For a more up-to-date gzweb visualization,
        see Option 2.

    1. Option 2: In a web browser, go to
       [https://app.gazebosim.org/visualization](https://app.gazebosim.org/visualization),
       and connect to the local websocket server on `ws://localhost:9002`.

# Authorization

The `websocket_server` plugin accepts to authentication keys:

* `<authorization_key>` : If this is set, then a connection must provide the matching key using an "auth" call on the websocket.
If the `<admin_authorization_key>` is set, then the connection can provide that key.

* `<admin_authorization_key>` : If this is set, then a connection must provide the matching key using an "auth" call on the websocket.
If the `<authorization_key>` is set, then the connection can provide that key.

Two keys are used in order to support authorization of different users.
A competition scenario may require admin access while prohibiting user
access.

# SSL

1. Use the `localhost.cert` and `localhost.key` files for testing purposes.
Configure the websocket server system using:

```xml
  <ssl>
    <cert_file>PATH_TO_localhost.cert</cert_file>
    <private_key_file>PATH_TO_localhost.key</private_key_file>
  </ssl>
```

   * You can create your own self-signed certificates using the following
   command. Use "localhost" for the  "Common Name" question.

   ```bash
   openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout server.key -out server.cert
   ```

2. Run gz-sim with the websocket-server system

3. Run a browser, such as firefox, with the `index.html` file.

```bash
firefox index.html
```

4. Open another browser tab, and go to `https://localhost:9002`. Accept the
   certificate.

5. Refresh the `index.html` browser tab.
