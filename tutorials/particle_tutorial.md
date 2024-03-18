\page particle_emitter Particle Emitter

This tutorial shows how to use the particle emitter system to add and configure particle effects like smoke and fog in simulation. It also shows the effects that particles have on different types of sensors in Gazebo.

## Particle Emitter System

We will demonstrate the particle emitter system by using the [examples/worlds/particle_emitter.sdf](
https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/particle_emitter.sdf) world.

To be able to spawn particle emitters,  first you will need to include the particle emitter system as a plugin to the world in your SDF. The system does not take any arguments.

```xml
    <plugin
      filename="gz-sim-particle-emitter-system"
      name="gz::sim::systems::ParticleEmitter">
    </plugin>
```

Next, we can start adding particle emitter models into the world. In our example world, we include a [Fog Generator](https://app.gazebosim.org/OpenRobotics/fuel/models/Fog%20Generator) model from Gazebo Fuel:

```xml
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/fog generator</uri>
    </include>
```

Here is the content of the Fog Generator [model.sdf](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Fog%20Generator/1/files/model.sdf) file.

```xml
  <model name="fog_generator">
    <pose>0 0 0 0 -1.5707 0</pose>
    <static>true</static>
    <link name="fog_link">
      <particle_emitter name="emitter" type="box">
        <emitting>true</emitting>
        <size>10 10 0</size>
        <particle_size>1 1 1</particle_size>
        <lifetime>25</lifetime>
        <min_velocity>0.1</min_velocity>
        <max_velocity>0.2</max_velocity>
        <scale_rate>0.5</scale_rate>
        <rate>5</rate>
        <material>
          <diffuse>0.7 0.7 0.7</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/fog.png</albedo_map>
            </metal>
          </pbr>
        </material>
        <color_range_image>materials/textures/fogcolors.png</color_range_image>
      </particle_emitter>
    </link>
  </model>
```

The SDF 1.6+ specification supports having a `<particle_emitter>` SDF element as a child of `<link>`. The particle emitter itself has several properties that can be configured, see Gazebo Rendering's [particles tutorial](https://gazebosim.org/api/rendering/8/particles.html) for more details on these properties. In our Fog Generator model, we are using a box type particle emitter that covers a region size of 10 by 10m. By default, the particles are emitted in the `+x` direction, hence the model as a pitch rotation of -90 degrees to rotate the particle emitter so that the particles are emitted upwards in `+z`.

Let's launch the example world to see what it looks like.

```bash
gz sim -v 4 -r particle_emitter.sdf
```

You should see the fog slowly starting to appear from the ground plane in the world:

@image html files/particle_emitter/fog_generator.png

Next, try changing some properties of the particle emitter while the simulation is running. You can do this by publishing messages over Gazebo Transport. Try turning off the particle emitter by setting the `emitting` property to `false`. Make sure the simulation is running in order for this command to take effect.

```bash
gz topic -t /model/fog_generator/link/fog_link/particle_emitter/emitter/cmd -m gz.msgs.ParticleEmitter -p 'emitting: {data: false}'
```

Note the above command tells the particle emitter to stop emitting. It does not make all the particles disappear immediately. The particles that have already been emitted will naturally fade and disappear over the specified `lifetime`.

Turn particle emitter back on by setting the `emitting` property to `true`:

```bash
gz topic -t /model/fog_generator/link/fog_link/particle_emitter/emitter/cmd -m gz.msgs.ParticleEmitter -p 'emitting: {data: true}'
```

Here is an example command for changing the `rate` property of the particle emitter. This should make the particle emitter emit more particles per second, causing it to visually appear more dense.

```bash
gz topic -t /model/fog_generator/link/fog_link/particle_emitter/emitter/cmd -m gz.msgs.ParticleEmitter -p 'rate: {data: 100}'
```

## Particle Effects on Sensors

The particles are not only a visual effect in simulation, they also have an effect on sensors in simulation. Here are the sensor types and the effects the particle emitter have on them:

* `camera`: The particles are visible to a regular camera sensor
* `depth_camera`: The particles have a scattering effect on the depth data.
* `rgbd_camera`: The particles are visible in the RGB image and have a scattering effect on the depth data.
* `gpu_lidar`: The particles have a scattering effect on the lidar range readings.
* `thermal_camera`: The particles are not visible in the thermal camera image.

The gif below shows an [example world](https://gist.github.com/iche033/bcd3b7d3f4874e1e707e392d6dbb0aa0) with six different sensors looking at the fog generator with a rescue randy model inside the fog.

@image html files/particle_emitter/sensor_scatter_tutorial.gif

The two image displays on the left show the images from a regular camera sensor and RGB output of a RGBD camera. The two have very similar color image output that shows the fog in the camera view. To their right are the depth images produced by a depth camera and the depth output of the RGBD camera. The depth readings are noisy due to the particle scattering effect. You can also see that these sensors can partially see through the fog and detect the rescue randy inside it. The bottom left of the gif shows the lidar visualization; the range data are also affected by the scattering effect. Finally, the thermal camera image display on the bottom right shows that thermal cameras do not detect particles.

The sensor scattering effect can be configured by adding `<particle_scatter_ratio>` to the `<particle_emitter>` SDF element. This property determines the ratio of particles that will be detected by sensors. Increasing the ratio means there is a higher chance of particles reflecting and interfering with depth sensing, making the emitter appear more dense. Decreasing the ratio decreases the chance of particles reflecting and interfering with depth sensing, making it appear less dense.

See image below that reduces the particle scatter ratio to 0.1. The depth camera image, RGBD camera's depth image, and lidar's range data are noticeably less noisy than the gif above.

@image html files/particle_emitter/particle_scatter_ratio.png
