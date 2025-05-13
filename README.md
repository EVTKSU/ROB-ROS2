# ROB-ROS2

ROS 2 workspace for EVT's MINI-ROB autonomous vehicle project.

---

## Table of Contents

- [Docker](#docker)
- [ROS 2 Workspace Structure](#ros-2-workspace-structure)
- [Strict Folder Requirements](#strict-folder-requirements)
- [Creating a New Python Package](#creating-a-new-python-package)
- [Building and Sourcing the Workspace](#building-and-sourcing-the-workspace)
- [Best Practices](#best-practices)

---

## Docker

> **Note:** The Docker image may not work for certain (newer) Apple devices due to the Ubuntu install filetype.

### Build the Docker Image

1. Ensure Docker Engine is running in Docker Desktop.
2. Open a terminal, navigate to the directory containing your Dockerfile, and execute the following commands:

**Build the Docker container:**

```bash
docker build -t ros2_image .
```

**Run the Docker container:**

```bash
docker run -it ros2_image
```

*This setup creates a Docker image based on Ubuntu, installs necessary dependencies, and runs your ROS 2 installer script inside the container.*

**Type `exit` to leave the container in your terminal.**

---

## ROS 2 Workspace Structure

This repository uses a standard [colcon](https://colcon.readthedocs.io/en/released/) build system with the following ROS 2 workspace layout:

```
ROB-ROS2/                 # Repository root
├── src/                  # ROS 2 workspace source folder (strict requirement)
│   ├── demo_setups/      # Contains simple Python demo packages
│   │   ├── pub_sub_demo/ # Minimal pub/sub example
│   │   └── service_demo/ # Minimal service/client example
│   └── system_msgs/      # Shared custom message definitions
├── Docker/               # Docker environment setup
│   ├── Dockerfile
│   └── docker-compose.yml
├── README.md             # Project documentation
└── colcon.meta           # (Optional) colcon metadata for workspace configuration
```

---

## Strict Folder Requirements

ROS 2 workspaces have strict structure rules:

1. **All ROS 2 packages must reside inside the `src/` folder.**
2. Each package must contain:
   - `package.xml` — Defines dependencies and metadata.
   - `setup.py` (for Python) or `CMakeLists.txt` (for C++).
3. The workspace must be built from the root directory (same level as `src/`).

Failing to follow this structure may cause `colcon` to skip packages or fail silently.

---

## Creating a New Python Package

To create a new Python-based ROS 2 package:

```bash
cd ~/ROB-ROS2/src
ros2 pkg create --build-type ament_python my_new_package
```

This command generates the following structure:

```
my_new_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_new_package
├── my_new_package/
│   ├── __init__.py
│   └── my_node.py
└── launch/
    └── my_launch_file.launch.py
```

Ensure that:

- The `package.xml` file specifies all necessary dependencies.
- The `setup.py` and `setup.cfg` files are correctly configured for installation.
- The Python module directory (`my_new_package/`) contains your node scripts.

---

## Building and Sourcing the Workspace

From the root of your workspace (`ROB-ROS2/`):

```bash
source /opt/ros/<ros_distro>/setup.bash  # Replace <ros_distro> with your ROS 2 distribution (e.g., foxy, humble)
colcon build --symlink-install
source install/setup.bash
```

Now you can run your nodes:

```bash
ros2 run my_new_package my_node
```

Or manually echo/test topics:

```bash
ros2 topic list
ros2 topic echo /your_topic
```

---

## Best Practices

- **Workspace Structure:** Maintain a single workspace (`ROB-ROS2/`) with all packages inside the `src/` directory.
- **Package Creation:** Use `ros2 pkg create` to generate new packages, ensuring proper structure and configuration.
- **Version Control:** Keep Docker configurations and ROS 2 packages in separate directories to avoid conflicts.
- **Documentation:** Include a `README.md` in each package to describe its purpose and usage.
- **Source Management:** Avoid sourcing the workspace's `setup.bash` in your `.bashrc` to prevent environment conflicts. Instead, source it manually in each new terminal session.

---