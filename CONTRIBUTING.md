# Contributing to Ignition Robotics

Thank you for your interest in contributing to Ignition Robotics!

The following is a set of guidelines for contributing to Ignition Robotics
and its components, which are hosted in the [Ignition Robotics
Organization](https://bitbucket.org/ignitionrobotics) on Bitbucket. These
are mostly guidelines, not rules. Use your best judgment, and feel free to
propose changes to this document in a pull request.

#### Table of Contents

[Code of Conduct](#markdown-header-code-of-conduct)

[Project Design](#markdown-header-project-design)

  * [Repository List](#markdown-header-repository-list)

[How to Contribute](#markdown-header-how-to-contribute)

  * [Reporting Bugs](#markdown-header-reporting-bugs)
  * [Suggesting Enhancements](#markdown-header-suggesting-enhancements)
  * [Contributing Code](#markdown-header-contributing-code)

[Writing Tests](#markdown-header-writing-tests)

  * [Test Coverage](#markdown-header-test-coverage)

[Styleguides](#markdown-header-style-guides)

[Appendix](#markdown-header-appendix)

## Code of Conduct

This project and everyone participating in it is governed by the [Ignition
Robotics Code of Conduct](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/default/CODE_OF_CONDUCT.md). By participating, you are expected to uphold this
code. Please report unacceptable behavior at [https://ignitionrobotics.org/support](https://ignitionrobotics.org/support).

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
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem. See [Creating GIFs](#markdown-header-creating-gifs) for GIF creation utilities.
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

This section guides you through submitting an enhancement suggestion,
including completely new features and minor improvements to existing
functionality. Following these guidelines helps maintainers and the
community understand your suggestion and find related suggestions.

Before creating enhancement suggestions, please check [this
list](#markdown-header-before-submitting-an-enhancement-suggestion) as you
might find out that you don't need to create one. When you are creating an
enhancement suggestion, please [include as many details as
possible](#markdown-header-how-do-i-submit-a-good-enhancement-suggestion).
When filling in the issue form for an enhancement suggestion, include the
steps that you imagine you would take if the feature you're requesting
existed.

#### Before Submitting An Enhancement Suggestion

* **Check if you're using the latest software version**. A more recent version may contain your desired feature.
* **Check if there's already [a library](https://ignitionrobotics.org/libs) which provides that enhancement.**
* **Determine [which repository the enhancement should be suggested in](#markdown-header-repository-list).**
* **Perform a [cursory search](https://bitbucket.org/search?q=&account=%7Bdd3bee6b-ed2f-496a-9634-d99cf5144fc5%7D)** to see if the enhancement has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.
* **Ask on the [community forum](https://community.gazebosim.org) about your
feature.** Someone else might already have started, and you might be able to
help.

#### How Do I Submit A (Good) Enhancement Suggestion?

Enhancement suggestions are tracked as [Bitbucket
issues](https://confluence.atlassian.com/bitbucket/issue-trackers-221449750.html).  After you've determined [which repository](#markdown-header-repository-list)
your enhancement suggestion is related to, create an issue on that
repository and provide the following information:

* **Use a clear and descriptive title** for the issue to identify the suggestion.
* **Provide a step-by-step description of the suggested enhancement** in as many details as possible.
* **Provide specific examples to demonstrate the steps**. Include copy/pasteable snippets which you use in those examples, as [Markdown code blocks](https://confluence.atlassian.com/bitbucketserver/markdown-syntax-guide-776639995.html).
* **Describe the current behavior** and **explain which behavior you expected to see instead** and why.
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem. See [Creating GIFs](#markdown-header-creating-gifs) for GIF creation utilities.
* **Explain why this enhancement would be useful** to most users and isn't something that can or should be implemented as a separate application.
* **Specify which version of Ignition Robotics you're using.**
* **Specify the name and version of the OS you're using.**

### Contributing Code

We follow a development process designed to reduce errors, encourage
collaboration, and make high quality code. Review the following to
get aquainted with this development process.

1. **Read the [Reporting Bugs](#markdown-header-reporting-bugs) and [Suggesting Enhancements](#markdown-header-suggesting-enhancements)** sections first.

1. **Fork the Ignition library** you want to contribute to. This will create
   your own personal copy of the library. All of your development should
   take place in your fork.

1. **Choose a base branch.** If your changes will break API or ABI, then
   base your new branch off of `default`. If your changes don't break
   API/ABI and you would like them to be released to an existing release
   with major version `N`, then use branch `ign-<library>N` as the base.

1. **Work out of a branch** Always work out of a new branch, one that is not
   the default/master branch. This is a good habit to get in, and will make
   your life easier.

1. **Write your code.** This is the fun part.

1. **Write tests.** In most cases, a pull request will only be accepted if
   it has tests. See the [Writing Tests](#markdown-header-writing-tests)
  section below for more information.

1. **Resolve compiler warnings.** Code must have zero compile warnings, or at least make sure your pull request is not adding new warnings.

1. **Follow the [style guide](#markdown-header-style-guides).**

    Static code checking analyzes your code for bugs, such as potential memory
    leaks, and style. Most Ignition libraries use the `cppcheck` static code
    checker, and a modified version `cpplint`. Ubuntu
    users can install via:

        sudo apt-get install cppcheck

    To check your code, run the following script from your `build` folder :

        make codecheck

    This may take a few minutes to run. Fix all errors and warnings until
    the output looks like:

        Built target codecheck

    The tool does not catch all style errors. See the [code style](#markdown-header-style-guides) section below for more information.

1. **(optional) Use clang-tidy for additional checks.**

    clang-tidy should return no errors when run against the code base.

    Ubuntu users can install via:

        sudo apt-get install clang-tidy-6.0 libclang-6.0-dev python-yaml

    clang-tidy can then be executed by running from the source dir:

        ./tools/clang_tidy.sh

    Address issues that are found.

    If you are feeling adventurous, you can experiment with adding additional checks to the `.clang-tidy` file by referencing the full list of options in the [clang-tidy documentation](http://clang.llvm.org/extra/clang-tidy/checks/list.html)

1. **Tests must pass.** You can check by running `make test` in
    your build directory. Running tests may take a bit of time, be patient.

1. **Write documentation.** Document all your code. Every class, function, member variable must have doxygen comments. All code in source files must have documentation that describes the functionality. This will help reviewers and future developers.

1. **Review your code.** Before submitting your code through a pull request,
   take some time to review everything line-by-line. The review process will
   go much faster if you make sure everything is perfect before other people
   look at your code.  There is a bit of the human-condition involved here.
   Folks are less likely to spend time reviewing your code if it's sloppy.

1. **Make small pull requests**

    A large pull request is hard to review, and will take a long time. It is
    worth your time to split a large pull request into multiple smaller pull
    requests. For reference, here are a few examples:

    * [Small, very nice](https://bitbucket.org/osrf/gazebo/pull-request/1732)

    * [Medium, still okay](https://bitbucket.org/osrf/gazebo/pull-request/1700/)

    * [Too large](https://bitbucket.org/osrf/gazebo/pull-request/30)

1. **Submit a pull request** to the Ignition library through Bitbucket when you're ready.

1. **Check Continuous integration**

    The moment you make a pull request, a few jobs in our
    [continuous integration](http://build.osrfoundation.org/)
    server will be started. These jobs will build your branch on Linux, Mac and
    Windows, run all tests and check for warnings.

    Your pull request will be updated with the status of these builds. Take some
    time to check these builds and see if you've introduced test failures,
    warnings or broke some build. If you did and know how to fix it, do so. If
    you don't know, speak up and someone may try to help you.

1. **Respond to reviewers.** At least two other people have to approve your pull request before it can be merged. Please be responsive to any questions and comments.

1. **Done, phew.** Once you have met all the requirements, you're code will be merged. Thanks for improving Ignition Robotics!

## Writing Tests

Most Ignition libraries use [GTest](http://code.google.com/p/googletest) for
general testing and [QTest](http://doc.qt.io/qt-5/qtest.html) for GUI tests.
There are a few kinds of tests:

1. **Unit tests**: all classes should have corresponding unit tests. These live
in the same directory as the source code and are suffixed by `_TEST`.

1. **Integration tests**: tests which verify how many classes are working together go under the `tests/integration` directory.

1. **Regression tests**: tests which fix broken features go under `tests/regression` and are prefixed by the issue number on librarie's issue tracker.

1. **Performance tests**: tests that are designed to check performance
   characterics, such as CPU or memory usage, go unde `tests/performance`.

Before creating a new integration or performance test file, check the current
test files. If one closely matches the topic of your new code, simply add a new
test function to the file. Otherwise, create a new test file, and write your
test.

### Test Coverage

The goal is to achieve 100% line and branch coverage. However, this is not
always possible due to complexity, analysis tools misreporting
coverage, and time constraints. Try to write as complete of a test suite as
possible, and use the coverage analysis tools as guide. If you have trouble
writing a test please ask for help in your pull request.

Ignition CMake provides build target called `make coverage` that produces a code
coverage report. You'll need to have [lcov](http://ltp.sourceforge.net/coverage/lcov.php) and [gcov](https://gcc.gnu.org/onlinedocs/gcc/Gcov.html) installed.

1. In your `build` folder, compile with `-DCMAKE_BUILD_TYPE=Coverage`

        cmake -DCMAKE_BUILD_TYPE=Coverage ..\
        make

1. Run a single test, or all the tests

        make test

1. Make the coverage report

        make coverage

1. View the coverage report

        firefox coverage/index.html

## Style Guides

You can check code for compliance by running the following command from
a build folder:

        make codecheck

In general, we follow [Google's style guide](https://google.github.io/styleguide/cppguide.html) and rules set forth by `cppcheck`. However, we have added some extras listed below.

1. **This pointer**
> All class attributes and member functions must be accessed using the `this->` pointer. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/src/Server.cc?at=default&fileviewer=file-view-default#Server.cc-112).

1. **Underscore function parameters**
> All function parameters must start with an underscore. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/src/Server.cc?at=default&fileviewer=file-view-default#Server.cc-130).

1. **Do not cuddle braces**
> All braces must be on their own line. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/src/Server.cc?at=default&fileviewer=file-view-default#Server.cc-159).

1. **Multi-line code blocks**
> If a block of code spans multiple lines and is part of a flow control statement, such as an `if`, then it must be wrapped in braces. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/src/Server.cc?at=default&fileviewer=file-view-default#Server.cc-203)

1. **++ operator**
> This occurs mostly in `for` loops. Prefix the `++` operator, which is [slightly more efficient than postfix in some cases](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/src/EntityComponentManager_TEST.cc?at=default&fileviewer=file-view-default#EntityComponentManager_TEST.cc-131).

1. **PIMPL/Opaque pointer**
> If you are writing a new class, it must use a private data pointer. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/include/ignition/gazebo/EntityComponentManager.hh?at=default&fileviewer=file-view-default#EntityComponentManager.hh-797), and you can read more [here](https://en.wikipedia.org/wiki/Opaque_pointer).

1. **const functions**
> Any class function that does not change a member variable should be marked as `const`. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/include/ignition/gazebo/EntityComponentManager.hh?at=default&fileviewer=file-view-default#EntityComponentManager.hh-340).

1. **const parameters**
> All parameters that are not modified by a function should be marked as `const`. This applies to parameters that are passed by reference, and pointer. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/include/ignition/gazebo/EntityComponentManager.hh?at=default&fileviewer=file-view-default#EntityComponentManager.hh-362).

1. **Pointer and reference variables**
> Place the `*` and `&` next to the varaible name, not next to the type. For example: `int &variable` is good, but `int& variable` is not. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/include/ignition/gazebo/EntityComponentManager.hh?at=default&fileviewer=file-view-default#EntityComponentManager.hh-362).

1. **Camel case**
> In general, everything should use camel case. Exceptions include SDF element names, and protobuf variable names. Here is an [example](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/3f2be4899c01e60bf5bb323812c281302d538c98/src/EntityComponentManager.cc?at=default&fileviewer=file-view-default#EntityComponentManager.cc-40).

1. **Class function names**
> Class functions must start with a capital letter, and capitalize every word.
>
> `void MyFunction();` : Good
>
> `void myFunction();` : Bad
>
> `void my_function();` : Bad

1. **Variable names**
> Variables must start with a lower case letter, and capitalize every word thereafter.
>
> `int myVariable;` : Good
>
> `int myvariable;` : Bad
>
> `int my_variable;` : Bad

1. **No inline comments**
> `//` style comments may not be placed on the same line as code.
>
> `speed *= 0.44704;  // miles per hour to meters per second` : Bad

## Appendix

### Creating GIFs

You can use [LICEcap](https://www.cockos.com/licecap/) to record GIFs on
macOS and Windows, and [Silent
Cast](https://github.com/colinkeenan/silentcast) or
[Byzanz](https://github.com/GNOME/byzanz) or
[Peek](https://github.com/phw/peek) on Linux. GIFs and videos are limited to
<= 1MB on Bitbucket. You can also upload media to other hosting services and
include a link on the issue.
