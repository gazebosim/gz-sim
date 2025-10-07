\page hpc_clusters Gazebo on HPC clusters

## Overview

This tutorial should help debugging and resolving the following problem:
**I run Gazebo via Apptainer/Singularity on an HPC cluster and adding
rendering sensors like cameras leads to a crash or they do not work (provide
no or black output).**

## Prerequisites

You have a Singularity/Apptainer image with Gazebo (and possibly ROS 2).
Let's call it `image.sif`.

You work on an HPC (High-performance Computing) cluster such as university or
national HPC centers. These clusters usually do not support Docker for security
reasons and support Singularity/Apptainer instead. Some ideas from this answer
may be applicable to Docker, too, but it is not a goal of this answer to
provide support for Docker users.

Apptainer == Singularity for this tutorial. There are minor differences between
reasonably new versions of these frameworks - but these are not important here
(Singularity at least version 3.x, Apptainer any version). Since Apptainer
installs command `singularity` for backwards compatibility, we'll be using
`singularity` command here, but Apptainer users can substitute it for
`apptainer` if they want.

Your HPC cluster has nodes with GPUs and you have access to them. These GPUs
are capable of rendering. The author of this tutorial is not sure if the
specialized "AI accelerators" like AMD MI300X and similar are capable of
rendering at all. But the normal NVidia datacenter cards like A100, B200, V100
and similar are okay. Check the GPU type with `nvidia-smi` command run on a GPU
node.

The HPC cluster runs some kind of task allocator that can give you time-limited
access to some node with requested resources. The tutorial author's cluster
uses SLURM, so we'll use SLURM commands here (`salloc`, `srun` etc.). But
theoretically, this guide should be independent from the task allocation engine.
For debugging purposes, it is better if the cluster provides interactive
sessions. But even if it only supports batch sessions, this should be doable,
it would just take more time to debug via batch jobs.

Let's assume the cluster partition with GPUs and interactive access is called
`gpufast` and the partition with non-interactive GPU access is `gpu`.

We also assume the home directory of your user is shared between the cluster
login node and the compute nodes. I.e. all files in your home folder are
accessible both from the login node and from within allocated tasks running on
nodes.

Your Gazebo world may be using either `ogre` or `ogre2` render engines, but
`ogre2` provides more options, so it is suggested to use that. It should be
configured
[in the Sensors system](https://github.com/gazebosim/gz-sim/blob/f67671cb3c6b309f14a04209edb94dc3fdc53b65/examples/worlds/sensors_demo.sdf#L15-L19).
You can also pass `--render-engine ogre2` on the Gazebo command line to
override the value specified in the world file.

Gazebo can render using 2 backends on Linux: **GLX** or **EGL**. GLX is the
older one, supported by both `ogre` and `ogre2` render engines. GLX needs a
running X11 server and it uses its GPU to do the rendering. Running a
HW-accelerated X11 server yourself is a bit nontrivial, but doable, as we'll
show later. EGL is the newer backend and it is only supported by `ogre2` render
engine and only for server-side rendering (i.e. it cannot be used for GUI). EGL
is selected by the `--headless-rendering` flag of Gazebo server. EGL works with
devices `/dev/dri/card?` and `/dev/dri/renderD???`. Each pair of these files
belongs to one GPU. Gazebo uses an autoselection algorithm to choose which GPU
will be used (it takes the first one capable of creating an EGL context). This
guide shows how the GPU can be selected explicitly.

To use EGL, you need read-write access to both `/dev/dri/card?` and
`/dev/dri/renderD???` devices. To check that, run the following command:

```bash
user@cluster$ srun -p gpufast--gres=gpu:1 --pty bash -i
user@gpu-node-1$ for f in /dev/dri/*; do [ -r "$f" ] && [ -w "$f" ] && echo "OK  $f" || echo "NOK $f"; done
```

Even if you see some NOKs, you can try following this guide. SLURM uses some
other magic to allow access (cgroups).

\note All the following approaches use `gz sim -s` command to test the
      rendering. It might be easier for you to test it with `glxgears` instead,
      which is a simple binary that should just show rotating cog wheels
      rendered by the GPU. This will work with all further described
      approaches, except approach 3 (`--headless-rendering`).
      To use `glxgears`, you need to install package `mesa-utils` (on
      Debian-based systems) or `glx-utils` (on Fedora-based systems) inside
      your image. Then, instead of `gz sim -v4 -r -s sensors_demo.sdf`, type
      `glxgears -info | cut -c -80` and look at the value of `GL_RENDERER`.
      `llvmpipe` is software rendering (slow), anything else should be GPU-accelerated.

\note The approaches are ordered by the simplicity of their use/setup if they
      work. However, most HPC clusters will probably not support
      Approach 1 and 2, so you can also start with Approach 3 first and then go
      to 1 and 2 if it doesn't work.

## Approach 1: Initial test

\note **Do not try to run GUI on the cluster unless the server-only part
      works**. It would only complicate things. To run only Gazebo server, make
      sure flag `-s` is in the `gz sim` command line (and `-g` is not).

*This tests Gazebo in GLX mode connected to an existing X server.*

Try directly running Gazebo on the GPU node. For easier testing and to rule-out
ROS complexity, use the `sensors_demo.sdf` world that comes preinstalled with
Gazebo. If you don't have it, just
[download it](https://github.com/gazebosim/gz-sim/blob/gz-sim10/examples/worlds/sensors_demo.sdf)
e.g. to your home folder and provide an absolute path to the file instead of
simply `sensors_demo.sdf`.

```bash
user@cluster$ srun -p gpufast--gres=gpu:1 --pty bash -i
user@gpu-node-1$ singularity exec image.sif bash -c "echo $DISPLAY; gz sim -v4 -s -r sensors_demo.sdf & (gz topic -t /camera -e | cut -c -80) & sleep 30; kill %1; kill %2; sleep 2; kill -9 %1; kill -9 %2"
```

This tests Gazebo on the interactive GPU partition. Now, let's do the same test
on a noninteractive one. Prepare the following job spec into file `test.batch`:

```bash
#!/bin/sh
#SBATCH --time=1 -p gpu --gres=gpu:1
singularity exec image.sif bash -c "gz sim -v4 -s -r sensors_demo.sdf & (gz topic -t /camera -e | cut -c -80) & sleep 30; kill %1; kill %2; sleep 2; kill -9 %1; kill -9 %2"
```

Of course, replace `-p gpu` with the name of your non-interactive GPU
partition.

Schedule the batch for execution and wait until it is executed. Check
`test.stdout` file for the output.

```bash
user@cluster$ sbatch -n1 -o test.stdout test.batch
user@cluster$ tail -F test.stdout
```

If the commands did not fail, did not print anything red and you see camera
messages printed out, rendering is working for you. Now it's time to find out
if it is HW-accelerated:

```bash
user@cluster$ grep -C2 RENDERER ~/.gz/rendering/ogre2.log
```

If you see `llvmpipe` in the output, the rendering is not HW-accelerated and
you should keep following this guide (unless you're happy with slower
rendering). If the output contains `NVidia` (or `AMD`), tadaa, you have a
working HW-accelerated rendering.

But usually, you'll see some kind of error like:

```
[Err] [Ogre2RenderEngine.cc:342] Unable to open display:
terminate called after throwing an instance of 'Ogre::RenderingAPIException'
  what():  OGRE EXCEPTION(3:RenderingAPIException): Couldn't open X display  in GLXGLSupport::getGLDisplay at /var/lib/jenkins/workspace/ogre-2.1-debbuilder/repo/RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXGLSupport.cpp (line 789)
```

**Explanation why this worked:** The cluster probably starts an X server for
each GPU node allocation and gives you access to this X server (via `$DISPLAY`
variable). Not many clusters do this. Some do it e.g. only for the interactive
partitions (like the OP's one) so the first set of commands works, but the
latter does not.

## Approach 2: Add \--nv switch

*This tests Gazebo in GLX mode connected to an existing X server, passing
--nv to Singularity.*

Now it's time to get a bit further and try running Singularity with the `--nv`
switch which basically adds some GPU-related host libraries to the container.
If you're on AMD GPUs, you should use `--rocm` instead, but that is not tested.
Contributions are welcome!

Test with almost the same command, just add `--nv` to singularity command:

```bash
user@cluster$ srun -p gpufast--gres=gpu:1 --pty bash -i
user@gpu-node-1$ singularity exec --nv image.sif bash -c "echo $DISPLAY; gz sim -v4 -s -r sensors_demo.sdf & (gz topic -t /camera -e | cut -c -80) & sleep 30; kill %1; kill %2; sleep 2; kill -9 %1; kill -9 %2"
```

Also, try it on the non-interactive partition, adjusting `test.batch`
appropriately.

What can happen if the node's OS is much newer than the OS inside your image,
is you'll get errors with GLIBC version mismatch. See
https://github.com/apptainer/apptainer/issues/945#issuecomment-2096052536 for a
workaround if that's your case.

If both interactive and non-interactive commands work for you in this case, you
are done (again, verify `RENDERER` in `ogre2.log`).

**Explanation why this worked:** Most Singularity images do not install
proprietary NVidia drivers inside. Instead, they rely on the host drivers, and
the `--nv` switch bind-mounts all files related to the host drivers inside the
container. So if there is a running X11 server, the programs from container
can use it.

## Approach 3: Try \--headless-rendering

*This tests Gazebo in EGL mode using autodetected GPU, passing --nv to
Singularity.*

This is the mode described in tutorial \ref headless_rendering.

If GLX does not work, let's try EGL backend:

```bash
user@cluster$ srun -p gpufast --gres=gpu:1 --pty bash -i
user@gpu-node-1$ singularity exec --nv image.sif bash -c "gz sim -v4 -s -r --headless-rendering sensors_demo.sdf & (gz topic -t /camera -e | cut -c -80) & sleep 30; kill %1; kill %2; sleep 2; kill -9 %1; kill -9 %2"
```

And, again, try the same on a non-interactive partition.

Check `RENDERER` in `ogre2.log` again.

You can also have a more detailed look into `~/.gz/rendering/ogre2.log`.
It contains two parts. In the first part, Gazebo is probing the
available GLX and EGL rendering devices. Here you can see whether Gazebo
correctly detects your GPU. You can ignore the GLX errors as we've
verified GLX doesn't work. Also, some EGL devices might occur multiple
times, e.g. in this example `/dev/dri/card2` appears as
`EGL_NV_device_cuda` and `EGL_EXT_device_drm`.
It is okay if just one of the two "views" of the device work.

```bash
01:13:59: OpenGL 3+ Rendering Subsystem created.
01:13:59: OGRE EXCEPTION(3:RenderingAPIException): Couldn't open X display  in GLXGLSupport::getGLDisplay at ./.obj-x86_64-linux-gnu/gz_ogre_next_vendor-prefix/src/gz_ogre_next_vendor/RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXGLSupport.cpp (line 808)
01:13:59: GLX raised an exception. Won't be available. Is X11 running?
01:13:59: OGRE EXCEPTION(3:RenderingAPIException): Couldn't open X display  in GLXGLSupport::getGLDisplay at ./.obj-x86_64-linux-gnu/gz_ogre_next_vendor-prefix/src/gz_ogre_next_vendor/RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXGLSupport.cpp (line 808)
01:13:59: Found Num EGL Devices: 5
01:13:59: EGL Device: EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #0 /dev/dri/card2
01:13:59: Trying to init device: EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #0 /dev/dri/card2...
01:13:59: Created GL 4.5 context for device EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #0 /dev/dri/card2
01:13:59: Destroying device: EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #0 /dev/dri/card2...
01:13:59: EGL Device: EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #1 /dev/dri/card3
01:13:59: Trying to init device: EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #1 /dev/dri/card3...
01:13:59: Created GL 4.5 context for device EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #1 /dev/dri/card3
01:13:59: Destroying device: EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #1 /dev/dri/card3...
01:13:59: EGL Device: EGL_EXT_device_drm EGL_EXT_device_drm_render_node #2 /dev/dri/card2
01:13:59: Trying to init device: EGL_EXT_device_drm EGL_EXT_device_drm_render_node #2 /dev/dri/card2...
01:13:59: OGRE EXCEPTION(3:RenderingAPIException): eglInitialize failed for device EGL_EXT_device_drm EGL_EXT_device_drm_render_node #2 /dev/dri/card2 in EGLSupport::getGLDisplay at ./.obj-x86_64-linux-gnu/gz_ogre_next_vendor-prefix/src/gz_ogre_next_vendor/RenderSystems/GL3Plus/src/windowing/EGL/PBuffer/OgreEglPBufferSupport.cpp (line 320)
01:13:59: OGRE EXCEPTION(3:RenderingAPIException): eglInitialize failed for device EGL_EXT_device_drm EGL_EXT_device_drm_render_node #2 /dev/dri/card2 in EGLSupport::getGLDisplay at ./.obj-x86_64-linux-gnu/gz_ogre_next_vendor-prefix/src/gz_ogre_next_vendor/RenderSystems/GL3Plus/src/windowing/EGL/PBuffer/OgreEglPBufferSupport.cpp (line 320)
01:13:59: Destroying device: EGL_EXT_device_drm EGL_EXT_device_drm_render_node #2 /dev/dri/card2...
01:13:59: EGL Device: EGL_EXT_device_drm EGL_EXT_device_drm_render_node #3 /dev/dri/card3
01:13:59: Trying to init device: EGL_EXT_device_drm EGL_EXT_device_drm_render_node #3 /dev/dri/card3...
01:13:59: OGRE EXCEPTION(3:RenderingAPIException): eglInitialize failed for device EGL_EXT_device_drm EGL_EXT_device_drm_render_node #3 /dev/dri/card3 in EGLSupport::getGLDisplay at ./.obj-x86_64-linux-gnu/gz_ogre_next_vendor-prefix/src/gz_ogre_next_vendor/RenderSystems/GL3Plus/src/windowing/EGL/PBuffer/OgreEglPBufferSupport.cpp (line 320)
01:13:59: OGRE EXCEPTION(3:RenderingAPIException): eglInitialize failed for device EGL_EXT_device_drm EGL_EXT_device_drm_render_node #3 /dev/dri/card3 in EGLSupport::getGLDisplay at ./.obj-x86_64-linux-gnu/gz_ogre_next_vendor-prefix/src/gz_ogre_next_vendor/RenderSystems/GL3Plus/src/windowing/EGL/PBuffer/OgreEglPBufferSupport.cpp (line 320)
01:13:59: Destroying device: EGL_EXT_device_drm EGL_EXT_device_drm_render_node #3 /dev/dri/card3...
01:13:59: EGL Device: EGL_MESA_device_software EGL_EXT_device_drm_render_node #4
01:13:59: Trying to init device: EGL_MESA_device_software EGL_EXT_device_drm_render_node #4...
01:13:59: Created GL 4.5 context for device EGL_MESA_device_software EGL_EXT_device_drm_render_node #4
01:13:59: Destroying device: EGL_MESA_device_software EGL_EXT_device_drm_render_node #4...
01:13:59: Plugin successfully installed
```

In the second part, you should see `Starting EGL Subsystem`. This
confirms Gazebo uses the EGL backend. Right beneath this line, you
should see which device is used for rendering:

```bash
01:13:59: ******************************
*** Starting EGL Subsystem ***
******************************
01:13:59: GL3PlusRenderSystem::_createRenderWindow "OgreWindow(0)_0", 1x1 windowed  miscParams: FSAA=0 border=none contentScalingFactor=1.000000 gamma=Yes parentWindowHandle=0 stereoMode=Frame Sequential
01:13:59: Trying to init device: EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #0 /dev/dri/card2...
01:13:59: Created GL 4.5 context for device EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node EGL_EXT_device_query_name EGL_EXT_device_persistent_id #0 /dev/dri/card2
01:13:59: GL Version = 4.5.0.0
01:13:59: GL_VERSION = 4.5.0 NVIDIA 550.120
01:13:59: GL_VENDOR = NVIDIA Corporation
01:13:59: GL_RENDERER = NVIDIA GeForce RTX 3090/PCIe/SSE2
```

If you see `EGL_MESA_device_software` or `llvmpipe` in this part,
it means EGL works, but is not HW-accelerated. Look in the first
section with probing to find out what problems are preventing to
use a HW device.

**Explanation why this worked:** `--headless-rendering` switches Gazebo to
the EGL backend.

## Approach 4: Use VirtualGL and a dummy X server

*This tests Gazebo in GLX mode using a selected GPU, connected to a dummy
X server.*

Generally, one of the 3 above approaches should work for you if your cluster
allows rendering at all. However, some GPUs have problems with some ways of
rendering, while working fine when done differently. Also, this allows you to
explicitly select the GPU to be used. Last, this method can be used to run
even Gazebo GUI on the cluster (if you e.g. want to record the GUI camera
programmatically).

First, you will need VirtualGL inside your image. Download a .deb or .rpm from
https://github.com/VirtualGL/virtualgl/releases and
`sudo apt install name_of.deb` or `sudo rpm -i name_of.deb`. You will also need
an X server. You can install `Xvfb` (`sudo apt install xvfb`), which only
provides the rendering backbone, but the actual rendered on-screen images are
thrown away (sensor rendering happens in off-screen buffers, so sensors are not
affected). Or you can install TurboVNC (from
https://github.com/TurboVNC/turbovnc/releases), which is an X server that
provides a VNC interface to which you can connect with any remote desktop
client. Also install `mesa-utils`/`glx-utils` if you haven't yet.

Rebuild the image and you should be able to run this command:

```bash
user@cluster$ srun -p gpufast--gres=gpu:1 --pty bash -i
user@gpu-node-1$ singularity exec image.sif xvfb-run -a glxgears -info | cut -c -80
```

This does not yet mean Gazebo HW-accelerated rendering works, because this only
tests software (non-accelerated) rendering.

Now you have to figure out which of the `/dev/dri/card?` EGL devices is the
card allocated to you by the cluster. The easiest thing is just trial and error
(you can use any of the card devices for which the command does not fail):


```bash
user@cluster$ srun -p gpufast--gres=gpu:1 --pty bash -i
user@gpu-node-1$ singularity exec --nv image.sif bash -c 'for f in /dev/dri/card*; do vglrun +v -d "$f" xvfb-run -a glxgears -info | cut -c -80; done'
```

Of course, there are more sophisticated methods. Here is an example Bash script
that translates IDs of the cards as shown by nvidia-smi to the `card?` and
`renderD???` devices:

```bash
#!/bin/bash -e

NVIDIA_SMI_ID=${1:-0}

minor=$(nvidia-smi -q -x -i $NVIDIA_SMI_ID 2>/dev/null | grep minor | cut -d \> -f2 | cut -d \< -f1)
if [ -z "$minor" ]; then
  echo "NVIDIA_SMI_ID=${NVIDIA_SMI_ID} does not denote an accessible GPU. Please, pass one of the IDs displayed by nvidia-smi." >&2
  exit 1
fi

echo "NVIDIA_SMI_ID=${NVIDIA_SMI_ID}"
pci_id=$(nvidia-smi -i $NVIDIA_SMI_ID --query-gpu=pci.bus_id --format=noheader,csv | tail -c+5 | tr '[:upper:]' '[:lower:]')

nvidia=/dev/nvidia$minor
card=$(ls /sys/bus/pci/devices/${pci_id}/drm/ | grep card)
render=$(ls /sys/bus/pci/devices/${pci_id}/drm/ | grep renderD)

echo "NVIDIA_DEVICE=$nvidia"
echo "DRI_DEVICE=$card"
echo "RENDER_NODE=$render"
```

If you find a working card (let's call it `cardN`), you can run Gazebo (do not
logout from the GPU node, otherwise the number of the card can change!) :

```bash
# on the same GPU node where you determined the available card
user@gpu-node-1$ singularity exec --nv image.sif bash -c "vglrun +v -d /dev/dri/cardN xvfb-run -a gz sim -v4 -s -r sensors_demo.sdf & (gz topic -t /camera -e | cut -c -80) & sleep 30; kill %1; kill %2; sleep 2; kill -9 %1; kill -9 %2"
```

Testing on the non-interactive partition looks the same, but you have to figure
out the available card device automatically, e.g. using the Bash script above.

If you want to actually see the graphical output (if running Gazebo with GUI),
use TurboVNC instead of Xvfb (it will print the VNC port at the start):

```bash
# on the same GPU node where you determined the available card
user@gpu-node-1$ singularity exec --nv image.sif /opt/TurboVNC/bin/vncserver -fg -log /dev/stdout -xstartup "vglrun +v -d /dev/dri/cardN gz sim -v4 -r sensors_demo.sdf & (gz topic -t /camera -e | cut -c -80) & sleep 30; kill %1; kill %2; sleep 2; kill -9 %1; kill -9 %2"
```

This, of course, assumes you have direct network access to the cluster nodes
and any ports you open on them.

Again, verify that Gazebo outputs no red text and check `RENDERER` in
`ogre2.log`.

**Explanation why this worked:** VirtualGL redirects GLX calls to a specified
EGL device (using some very low-level tricks). However, the app (Gazebo) still
thinks it's using GLX, so it needs an X server. That's the place for `xvfb` or
`TurboVNC`.

## Approach 5: \--nv is broken, but there is hope

On some systems, the `--nv` switch does not add all files necessary for working
with VirtualGL inside a container. You can try to fix that and bind-mount these
manually. This should help with approaches 2, 3 and 4.

Someone had to mount these in addition to the default:

```bash
--bind /usr/lib64/libEGL_nvidia.so.0 --bind /usr/share/glvnd/egl_vendor.d/
```

It seems that Apptainer 1.4.0 and newer
[should already have this fixed](https://github.com/apptainer/apptainer/pull/2572).
But if you have an older version, these manual binds will do the job.

It is possible that your system has other files that will need to be mounted.
You can try to search for them manually around the files that are mentioned in
`/etc/singularity/nvliblist.conf`. Or, if your cluster provides
`nvidia-container-cli` command, you can try to extract the list of files from
`nvidia-container-cli list`. But the author of the tutorial does not have
access to a cluster with this command, so this is untested (there is even
`--nvccli` flag in addition to `--nv` in Apptainer that could help with this).

### Low-Level Last Resort (only for adventurous or desperate)

Alternatively, you can utilize `strace` and try running VirtualGL directly on
the GPU node outside Singularity container. However, this approach is quite
complicated for many users.

If you can get `strace` working directly on the GPU node (not inside
Singularity), you can also unpack packages `mesa-utils`/`glx-utils`, `xvfb`
and `VirtualGL` to some of your writable locations and try to run them
directly on the node. This will rule-out problems introduced by Singularity.

First, download .deb or .rpm files of the packages for the version of OS that
is running on the GPU node (use `cat /etc/*-release` to figure it out). To
unpack a .deb file, run `dpkg-deb -xv file.deb .` . To unpack an .rpm file, run
`rpm2cpio file.rpm | cpio -idmv` or `rpm2cpio file.rpm | zstd -d | cpio -idmv` .

Next, add the unpacked lib and lib64 folders to `LD_LIBRARY_PATH` and bin
folders to `PATH`. Then you can use `strace`, `xvfb`, `virtualgl` and
`glxgears` to test which NVidia files are loaded:

```bash
LD_LIBRARY_PATH="`pwd`/usr/lib64:$LD_LIBRARY_PATH" PATH="`pwd`/usr/bin:$PATH" strace -f -e openat ./opt/VirtualGL/bin/vglrun +v -d /dev/dri/card1 xvfb-run -a glxgears -info 2>&1 | grep -v ENOENT | grep -i nv
```

The command may fail for various reasons, you might need to download recursive
dependencies and so on. If your cluster uses e.g. the Lmod modules system, you
can provide a part of the dependencies just by typing `module load X11`.
If you can get this working, you would get the definitive answer to which files
are needed inside the container. If not, you just have to guess.
</details>

**Explanation why this worked:** The list of files in `nvliblist.conf` is just
a guess. Some might be missing, some
[are there and shouldn't](https://github.com/apptainer/apptainer/issues/945#issuecomment-1374524681).
If you finish this list manually by adding `--bind` mounts of the missing
files, you will have a working GLX and EGL system inside the container.

## If Nothing Works

If nothing from the above works, it is possible that your cluster has just not
configured the GPU resources properly and it doesn't give you the permissions
to use the GPUs via the `/dev/dri/` files, but only through `/dev/nvidia?`.

You can try contacting your cluster admins and showing them this link with a
suggestion how to reconfigure the resources:
https://groups.google.com/g/slurm-users/c/n_oUtvdTC4o/m/r4abS8KMBAAJ . At the
time of writing that post, there was no folder `/dev/dri/by-path`. On recent
machines, this folder is present, so it could be even easier to define the
resources.
