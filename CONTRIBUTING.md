# Contributing to Ignition Robotics

Thank you for your interest in contributing to Ignition Robotics!

The following is a set of guidelines for contributing to Ignition Robotics
and its components, which are hosted in the [Ignition Robotics
Organization](https://bitbucket.org/ignitionrobotics) on Bitbucket. These
are mostly guidelines, not rules. Use your best judgment, and feel free to
propose changes to this document in a pull request.

## Code of Conduct

This project and everyone participating in it is governed by the [Ignition
Robotics Code of Conduct](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/default/CODE_OF_CONDUCT.md). By participating, you are expected to uphold this
code. Please report unacceptable behavior at [https://ignitionrobotics.org/support](https://ignitionrobotics.org/support).

\todo(nkoenig) Pull content from: https://bitbucket.org/osrf/gazebo/src/default/CONTRIBUTING.md

## Project Design

### Repository List

The following is a list of the major Ignition Robotics repositories. A complete list can be found at [https://bitbucket.org/ignitionrobotics/](https://bitbucket.org/ignitionrobotics/).

* [ign-cmake](https://bitbucket.org/ignitionrobotics/ign-cmake): CMake
modules used to configure and build Ignition libraries.
* [ign-common](https://bitbucket.org/ignitionrobotics/ign-common): Set of
general purpose components and utilities, such as 3D mesh processing, console
logging, and signal handling.
* [ign-gazebo](https://bitbucket.org/ignitionrobotics/ign-gazebo):
A high-fidelity 3D rigid-body dynamic simulator.
* [ign-gui](https://bitbucket.org/ignitionrobotics/ign-gui): QT-based
library to configure and manage graphical applications.
* [ign-math](https://bitbucket.org/ignitionrobotics/ign-math): A math
library targeted toward robotic applications.
* [ign-msgs](https://bitbucket.org/ignitionrobotics/ign-msgs): Protobuf
messages and utilities for simulation and robotics.
* [ign-physics](https://bitbucket.org/ignitionrobotics/ign-physics):
Plugin based library for physics simulation.
* [ign-plugin](https://bitbucket.org/ignitionrobotics/ign-plugin): Library
for registering, loading, and managing plugin libraries.
* [ign-rendering](https://bitbucket.org/ignitionrobotics/ign-rendering):
Library that supports rendering through different engines, such as
[OGRE](https://www.ogre3d.org/) and [Optix](https://developer.nvidia.com/optix).
* [ign-sensors](https://bitbucket.org/ignitionrobotics/ign-sensors): A set
of models that generate realistic sensor data. 
* [ign-tools](https://bitbucket.org/ignitionrobotics/ign-tools): Provides
the `ign` command line interface that can be configured and used by multiple
libraries. 
* [ign-transport](https://bitbucket.org/ignitionrobotics/ign-transport):
High performance inter- and intra-process communication based on
[ZeroMQ](http://zeromq.org/) and [Protobuf](https://developers.google.com/protocol-buffers/).

## How to Contribute

### Reporting Bugs

**Before Submitting a Bug Report**

1. Check the [questions and answers forum](http://answers.gazebosim.org). Your issue may have already been resolved.
2. Determine [the repository](#markdown-header-repository-list) which should receive the problem.
3. Search the repository's issues to see if the same or similar problem has
   been opened. If it has and the issue is still open, then add a comment to
   the existing issue. Otherwise, create a new issue.

**How to Submit a Good Bug Report**

Create an issue on the repository that is related to your bug, explain the
problem, and include additional details to help maintainers reproduce the
problem. Refer to the [Short, Self Contained, Correct (Compilable), Example
Guide](http://sscce.org/) as well as the following tips:

* **Use a clear and descriptive title** for the issue to identify the problem.
* **Describe the exact steps which reproduce the problem** in as many details as possible. When listing steps, don't just say what you did, but explain how you did it.
* **Provide specific examples to demonstrate the steps.** Include links to files or projects, or copy/pasteable snippets, which you use in those examples.
* **Describe the behavior you observed after following the steps** and point out what exactly is the problem with that behavior.
* **Explain which behavior you expected to see instead and why**.
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem. You can use [this tool](https://www.cockos.com/licecap/) to record GIFs on macOS and Windows, and [Silent Cast](https://github.com/colinkeenan/silentcast) or [Byzanz](https://github.com/GNOME/byzanz) or [Peek](https://github.com/phw/peek) on Linux. GIFs and videos are limited to <= 1MB on Bitbucket. You can also upload media to other hosting services and include a link on the issue.
* **If the problem wasn't triggered by a specific action**, describe what you were doing before the problem happened and share more information using the guidelines below.

Provide more context by answering these questions:

* **Did the problem start happening recently** (e.g. after updating to a new version) or was this always a problem?
* If the problem started happening recently, **can you reproduce the problem in an older version?** What's the most recent version in which the problem doesn't happen?
* **Can you reliably reproduce the issue?** If not, provide details about how often the problem happens and under which conditions it normally happens.
* If the problem is related to working with files, **does the problem happen for all files and projects or only some?** Does the problem happen only when working with local or remote files, with files of a specific type (e.g. only Collada or SDF), or with large files or files with very long lines? Is there anything else special about the files you are using?

Include details about your configuration and environment:

* **Which version of Ignition Robotics are you using?**?
* **What's the name and version of the OS you're using**?
* **Are you running Ignition in a virtual machine?** If so, which VM software are you using and which operating systems and versions are used for the host and the guest?

### Suggesting Enhancements

### Your First Code Contribution

### Pull Request Process

## Styleguides

