# Gazebo Sim Dockerfiles

This directory contains a few Dockerfiles and supporting scripts. See each
section below for usage information about

1. [Dockerfile.gz](#Gazebo-Sim-Using-Debians-In-Docker)
1. [Dockerfile.nightly](#Build-Gazebo-Sim-Using-Nightly-Debians)

## Build Gazebo Sim Using Nightly Debians

This section describes how to build and run a docker image based on
nightly builds of downstream
[Gazebo libraries](https://gazebosim.org/libs). The Docker image will
use the Gazebo code found in the current source tree.

**Requirements**

1. [Install Docker](#Install-Docker)

1. *Optional:* [Install NVidia Docker](#Install-Nvidia-Docker)

    Nvidia docker will be needed if you plan to run the GUI and/or sensors
    after the Docker image is built.

**Steps**

1. Change the root directory of the Gazebo Sim source tree. If you are
   currently in the `docker` subdirectory:

    ```
    cd ..
    ```

1. Build the gz-sim:base image.

    ```
    docker build . -f ./docker/Dockerfile.base -t gz-sim:base
    ```

2. Build the nightly docker image.

    ```
    docker build . -f ./docker/Dockerfile.nightly -t gz-sim:nightly
    ```

3. Run the docker image with a bash shell.

    ```
    docker run -it gz-sim:nightly /bin/bash
    ```

4. Alternatively, you can directly run Gazebo using

    ```
    ./docker/run.bash gz-sim:nightly gz-sim-server -v 4
    ```

## Gazebo Using Debians In Docker

This section describes how to build and run a docker image of a Gazebo
distribution using debians.

**Requirements**

1. [Install Docker](#Install-Docker)

2. [Install NVidia Docker](#Install-Nvidia-Docker)

**Steps**

1. Build a docker image using the `build.bash` command. The first argument
   must be the name of the Gazebo distribution. The list of Gazebo distribution can be found at [Gazebo distribution](https://gazebosim.org/docs). For example, to build an
   image of Gazebo Garden:

    ```
    ./build.bash gz-garden ./Dockerfile.gz
    ```

2. Run the docker image using `run.bash`, and pass in the name of the docker
   image (first argument to the build.bash script).

    ```
    ./run.bash gz-garden
    ```

3. You can pass arguments to Gazebo by appending them the
   `run.bash` command. For example, to load the shapes.sdf file:

    ```
    ./run.bash gz-garden -f shapes.sdf
    ```

## Appendix

Here you'll find general information, such as installation of Docker and how
to use Nvidia with Docker.

### Install Docker

Docker has two available versions: Community Edition (CE) and Enterprise Edition (EE). In this tutorial, we'll install the CE version.

1.  Remove old versions of Docker (if installed):

        sudo apt-get remove docker docker-engine docker.io

1. Install dependencies and keys.

        sudo apt install curl apt-transport-https ca-certificates curl software-properties-common

        # Add the official GPG key of Docker
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

1. Setup Docker.

        sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

1. Install Docker

        sudo apt-get update && sudo apt-get install docker-ce

1. Check your Docker installation:

        sudo docker run hello-world

1. You should see the message `Hello from Docker!` confirming that your installation was successfully completed.

### Install Nvidia Docker

1. Remove old versions of [Nvidia Docker](https://github.com/NVIDIA/nvidia-docker):

        docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
        sudo apt-get purge -y nvidia-docker

1. Setup the Nvidia Docker repository.

        curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
        distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
        curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
        sudo apt-get update

1. Install Nvidia Docker (version 2):

        sudo apt-get install -y nvidia-docker2

1. Restart the Docker daemon

        sudo service docker restart

1. Verify the installation:

        docker run --gpus all --rm nvidia/cuda nvidia-smi

    This command should print your GPU information, for example:

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 390.116                Driver Version: 390.116                   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  GeForce GTX 960     Off  | 00000000:01:00.0  On |                  N/A |
|  0%   51C    P5    12W / 160W |    775MiB /  2000MiB |      1%      Default |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
+-----------------------------------------------------------------------------+
```
