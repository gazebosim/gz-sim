---
name: Bug report
about: Report a bug
labels: bug
---

<!-- If you're not sure whether your problem is a bug, please ask a question at
https://robotics.stackexchange.com instead.-->

## Environment
* OS Version: <!-- e.g, Ubuntu 20.04 -->
* Source or binary build?
  <!-- If binary, which version? -->
  <!-- If source, which branch and what is the latest commit? -->
* If this is a GUI or sensor rendering bug, describe your GPU and rendering system. Otherwise delete this section. <!-- If you do not know some values, leave them out. But please, at least give a try to finding them - they are really helpful when debugging rendering errors. -->
    - Rendering plugin: [ogre | ogre2]. <!-- You can find it in the logs of Gazebo it outputs to console when you launch it with `-v 3` or `-v 4` argument. -->
      - [ ] Sensor rendering error. <!-- Search for message like "[Msg] Loading plugin [gz-rendering-ogre]" (not starting with "[GUI]"). -->
      - [ ] GUI rendering error. <!-- Search for message like "[GUI] [Msg] Loading plugin [gz-rendering-ogre2] -->
    - EGL headless mode:
      - [ ] Running in EGL headless mode <!-- only available since Gazebo Fortress, GPU display is specified as /dev/dri/card* instead of :0 -->
    - Generally, mention all circumstances that might affect rendering capabilities: <!-- remove lines that do not apply to keep the list short -->
      - [ ] running on a dual GPU machine (integrated GPU + discrete GPU)
      - [ ] running on a multi-GPU machine (it has multiple discrete GPUs)
      - [ ] running on real hardware
      - [ ] running in virtual machine
      - [ ] running in Docker/Singularity
      - [ ] running remotely (e.g. via SSH)
      - [ ] running in a cloud
      - [ ] using VirtualGL, XVFB, Xdummy, XVNC or other indirect rendering utilities
      - [ ] GPU is concurrently used for other tasks
        - [ ] desktop acceleration
        - [ ] video decoding (i.e. a playing Youtube video)
        - [ ] video encoding
        - [ ] CUDA/ROCm computations (Tensorflow, Torch, Caffe running)
        - [ ] multiple simulators running at the same time
      - [ ] other...
    - Rendering system info:
      - On Linux, provide the outputs of the following commands:
          ```bash
          LANG=C lspci -nn | grep VGA  # might require installing pciutils
          echo "$DISPLAY"
          LANG=C glxinfo -B | grep -i '\(direct rendering\|opengl\|profile\)'  # might require installing mesa-utils package
          ps aux | grep Xorg
          sudo env LANG=C X -version  # if you don't have root access, try to tell the version of Xorg e.g. via package manager
          ```
      - On Windows, run `dxdiag` and report the GPU-related information.
      - On Mac OS, open a terminal and type `system_profiler SPDisplaysDataType`. Copy the output here.
        <!-- Please note that GUI rendering is not supported on macOS. -->
    - [ ] Please, attach the ogre.log or ogre2.log file from  `~/.gz/rendering` <!-- Choose the relevant version based on what you checked in the "Rendering plugin" question. -->

<details>

```
# paste log here
```

</details>

## Description
* Expected behavior: <!-- Tell us what you expected to happen -->
* Actual behavior: <!-- What happened instead -->

## Steps to reproduce
<!-- Provide steps so we can try to reproduce this issue -->

1.
2.
3.

## Output
<!-- Provide screenshots, console logs, backtraces, and/or anything that could
be useful to us resolving this issue -->
