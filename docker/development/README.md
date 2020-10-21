# Docker for Ignition Gazebo Development

This directory contains files that allow users to use/develop/test source builds of [ign-gazebo](https://github.com/ignitionrobotics/ign-gazebo) inside of a Docker container.

There are three files in this directory:
* [`Dockerfile`](./Dockerfile): The Dockerfile that defines the image, which includes all of the `ign-gazebo` dependencies and build tools like [colcon](https://colcon.readthedocs.io/en/released/#).
* [`build.bash`](./build.bash): A bash script that builds the Docker image.
* [`run.bash`](./run.bash): A bash script that starts a Docker container, loading a user's copy of the `ign-gazebo` repository into the container at run time as a [volume](https://docs.docker.com/storage/volumes/).

## Requirements

* Docker (installation instructions for Ubuntu can be found [here](https://docs.docker.com/engine/install/ubuntu/))
* [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker)
* A computer with an Nvidia graphics card

## Usage

1. Clone the `ign-gazebo` repository (in this example, we are cloning the repository to the `$HOME` directory, but you can clone it to another location if you'd like):

```
cd ~/
git clone https://github.com/ignitionrobotics/ign-gazebo.git
```

2. Switch to the `docker/development` directory:

```
cd ign-gazebo/docker/development/
```

3. Build the Docker image with the `build.bash` script.
This script requires an image name and path to the [repository's `packages*.apt` files](https://github.com/ignitionrobotics/ign-gazebo/tree/main/.github/ci) as arguments (`packages*.apt` contains the `ign-gazebo3` dependencies).
The following command would create a Docker image named `gazebo3_devel`:

```
./build.bash gazebo3_devel ~/ign-gazebo/.github/ci/
```

4. Execute `run.bash` to start a container with graphics capabilities that also has the `ign-gazebo3` repository loaded as a volume.
The two parameters needed for this step are the name of the image (from the previous step) and the path to the `ign-gazebo3` repository.
If you went through steps 1-3 as shown above, run the following command:

```
./run.bash gazebo3_devel ~/ign-gazebo/
```

5. Once in the container, you can build `ign-gazebo3` with `colcon` (the container will start you in the root of the `ign-gazebo3` workspace by default):

```
colcon build
```

6. Source the workspace and start gazebo to verify that everything is working properly (if the `ign gazebo` command below doesn't work for you, try the fix in step 7):

```
. install/setup.bash
ign gazebo
```

7. If running `ign gazebo` in step 6 didn't work, you may need to manually specify where gazebo has been installed
(more information about this issue can be found [here](../../README.md#known-issue-of-command-line-tools)):

```
export IGN_CONFIG_PATH=~/ws/install/ignition-gazebo3/share/ignition
ign gazebo
```

That's it! 
You can now use your cloned repository of `ign-gazebo` to test or develop new features.

Since the repository was loaded into the container as a volume, you can make changes to the source code on your machine locally with your preferred development tools, and these changes will automatically be reflected inside of the Docker container.
Another nice thing about loading the repository as a volume is that any changes you make to the repository will persist after the container is shut down.
This allows for easy development and testing with Docker without having to set up development tools inside of Docker.