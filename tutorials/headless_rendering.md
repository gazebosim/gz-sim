\page headless_rendering Headless Rendering

It is often desirable to run simulation on a remote computer, such as
a computer managed by cloud provider, in order to paralellize work or access
specific compute resources. Simulated sensors that require GPU access have
historically been difficult to use on a remote computer due to OpenGL's
X server requirement on linux systems. This issue can be resolved through
installation and proper configuration of X, but the steps can be complex and
error prone.

An easier solution is through the use of [EGL](https://www.khronos.org/egl), which allows for the the creation of rendering surfaces without an X server. Gazebo has added support for EGL via the `--headless-rendering` command line option. Use of EGL is only available with OGRE2.

Example usage:

```
gz sim -v 4 -s --headless-rendering sensors_demo.sdf
```

If you are using Gazebo as a library, then you can configure the
server to use headless rendering through the
`ServerConfig::SetHeadlessRendering(bool)` function. Make sure your SDF
world uses OGRE2.

## AWS Example

This example will guide you through the process of launching and configuring
an AWS GPU instance with Gazebo running headless. A GPU instance is
recommended when sensors that require a render engine are used. It is
possible to use a machine without a GPU, in which case OGRE will revert to
software rendering. You can read more about [OGRE's EGL implementation
here](https://www.ogre3d.org/2021/02/06/ogre-2-2-5-cerberus-released-and-egl-headless-support).

1. Go to the [AWS EC2 service](https://console.aws.amazon.com/ec2)
2. Click the `Launch Instance` button in the upper right.
3. Select `Ubuntu Server` version 20.04 or greater from the AMI list.
4. Choose a GPU enabled instance type, such as `g3.4xlarge`.
5. Enable `Auto-assign Public IP` on the `Configure Instance Details` step.
   This is not the best practice, but it simplifies this tutorial.
6. Add around 200GB storage to your instance on the `Add Storage` step.
7. Enable ssh source `Anywhere` on the `Configure Security Group` step.
8. Review and launch your instance. Make sure to setup a key pair in the
   popup that appears after clicking `Launch`.
    1. You can configure other options as needed. Review the [AWS
   documentation](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/EC2_GetStarted.html) for additional help.
9. Select the newly launched instance on the EC2 dashboard, and take note of
   the `Public IPv4 address`.
10. SSH into your new machine instance.
  ```
  ssh -i SSH_PEM_FILE_USED_DURING_LAUNCH ubuntu@EC_INSTANCE_PUBLIC_IP
  ```
12. Install Ubuntu drivers, which will install nvidia drivers:
  ```
  sudo apt-get update
  sudo apt install ubuntu-drivers-common
  sudo ubuntu-drivers install
  ```
13. Add the `ubuntu` user to the `render` group, which is required to access
    to the dri interfaces.
  ```
  sudo usermod -a -G render ubuntu
  ```
14. Reboot the machine and log back in.
  ```
  sudo reboot
  ```
11. [Install Gazebo](https://gazebosim.org/docs/latest/install).
12. Run a Gazebo world that uses OGRE2 with camera sensors using headless rendering. This will enable EGL.
  ```
  gz sim -v 4 -s -r --headless-rendering sensors_demo.sdf
  ```
13. Check that simulation is producing sensor data by ssh'ing into the EC2
    instance from a new terminal and echoing a sensor topic.
  ```
  ssh -i SSH_PEM_FILE_USED_DURING_LAUNCH ubuntu@EC_INSTANCE_PUBLIC_IP
  gz topic -et /thermal_camera
  ```
