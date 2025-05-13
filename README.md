# ROB-ROS2
ROS2 workspace for EVT's MINI-ROB autonomous vehicle project

---

## Table of Contents
- [Docker](#docker)
- [ROS 2 Workspace Layout](#ros-2-workspace-layout)
- [Strict Folder Requirements](#strict-folder-requirements)
- [Creating a New Python Package](#creating-a-new-python-package)
- [Building and Sourcing](#building-and-sourcing)
- [Best Practices](#best-practices)

---

## Docker
> **Note:** The Docker image may not work for certain (newer) Apple devices due to the Ubuntu install filetype.

### Build the Docker Image:

1. Run Docker Engine in Docker Desktop.
2. Open a terminal, `cd` to the directory containing your Dockerfile and run these commands:

**Build the Docker container:**
```
docker build -t ros2_image .
```

**Run the Docker container:**
```
docker run -it ros2_image
```

*This setup creates a Docker image based on Ubuntu, installs necessary dependencies, and runs your ROS 2 installer script inside the container.*

**Type `exit` to leave the container in your terminal.**

---

## ROS 2 Workspace Layout

This repository uses a standard [colcon](https://colcon.readthedocs.io/en/released/) build system with the following ROS 2 workspace layout:

```
ros2-kart-stack/           # Repository root
├── src/                   # ROS 2 workspace source folder (strict!)
│   ├── demo_setups/       # Team training demos
│   │   ├── pub_sub_demo/  # Pub/sub minimal example
│   │   └── service_demo/  # Service/client minimal example
│   └── system_msgs/       # Custom shared messages (if any)
├── Docker/                # Docker environment setup
├── README.md              # You are here
```

---

## Strict Folder Requirements

ROS 2 workspaces have **strict structure rules**:

1. **All ROS 2 packages must be inside the `src/` folder.**
2. Each package must contain:
   - `package.xml` — defines dependencies and metadata
   - `setup.py` (for Python) or `CMakeLists.txt` (for C++)
3. The workspace must be built from the root (same level as `src/`).

Failing to follow this structure will cause `colcon` to **skip packages** or **fail silently**.

---

## Creating a New Python Package

To create a new Python-based ROS 2 package:

```bash
cd ~/ros2-kart-stack/src
ros2 pkg create --build-type ament_python my_new_package
```

Then inside `my_new_package/`, you'll need:

- `package.xml` → describe package + dependencies
- `setup.py` and `setup.cfg` → Python package setup
- `my_new_package/` folder → contains your Python nodes
- `launch/` (optional) → for launching multiple nodes

Example node location:
```
my_new_package/
├── my_new_package/
│   └── talker.py
```

---

## Building and Sourcing

From the root of the repo (above `src/`):

```bash
source /opt/ros/jazzy/setup.bash  # or humble/iron depending on ROS version
colcon build
source install/setup.bash
```

Now you can run your nodes:
```bash
ros2 run my_new_package talker
```

Or manually echo/test topics:
```bash
ros2 topic list
ros2 topic echo /your_topic
```

---

## Best Practices

- Keep Pi and Panda packages in the **same workspace** unless hardware constraints require otherwise.
- Use the `demo_setups/` folder for training, tutorials, and beginner onboarding.
- Document each package with a `README.md` inside its folder.
- Use `ros2 pkg create` for all new packages to avoid manual errors.
- Don't put ROS packages outside of `src/`—they won't be detected by `colcon`.

---
