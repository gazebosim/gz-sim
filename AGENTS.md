# Agent Instructions for Gazebo Sim (gz-sim)

Welcome, Agent. This repository contains the source code for Gazebo Sim. Please follow these guidelines to ensure consistency and compliance with project standards.

## 🛠 Building the Project

The best tested build environment is Ubuntu. For other platforms (macOS, Windows), please refer to the [official Gazebo documentation](https://gazebosim.org/docs/latest/).

### Ubuntu Source Installation

1.  **Add OSRF Repository:**
    ```bash
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    ```

2.  **Install Dependencies:**
    Run this command from your workspace root:
    ```bash
    sudo apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
    ```

3.  **Build with Colcon:**
    ```bash
    colcon build --merge-install
    ```
    *Note: Compilation can be resource-intensive. If you encounter RAM issues, use `export CMAKE_BUILD_PARALLEL_LEVEL=1` and add `--executor sequential` to the colcon command.*

## 📝 Contribution Guidelines

### Commits
All commits must follow these rules:
- **Sign-off:** Use `git commit -s` to add the `Signed-off-by` line.
- **Attribution:** Include a `Generated-By: <Agent Name>` line in the commit message footer.

### Pull Requests
When creating a Pull Request:
- **Template:** Always fill out the PR template: https://github.com/gazebosim/.github/blob/main/PULL_REQUEST_TEMPLATE.md
- **Description:** Provide a clear and concise description of the changes, the rationale behind them, and how they were tested.

## 🔍 Additional Resources
- [Gazebo Sim Documentation](https://gazebosim.org/docs/latest/)
- [Contribution Guide](https://gazebosim.org/docs/all/contributing)
