# pub_sub_demo

This is a minimal ROS 2 Python package that demonstrates publisher and subscriber communication using the `std_msgs/String` message type. It serves both as a beginner learning tool and a template for building future ROS 2 packages.

---

## What This Package Does

- `talker.py` – Publishes `"Hello, world!"` messages to the `/chatter` topic once per second.
- `listener.py` – Subscribes to `/chatter` and prints any received message to the console.
- `pub_sub_demo.launch.py` – Launches both nodes together using ROS 2's launch system.

---

## Package Structure

```
pub_sub_demo/
├── pub_sub_demo/
│   ├── talker.py               # The publisher node
│   └── listener.py             # The subscriber node
├── launch/
│   └── pub_sub_demo.launch.py  # Launch file for both nodes
├── package.xml                 # Package metadata
├── setup.py                    # Python setup script with node entry points
├── setup.cfg                   # Optional config metadata
├── resource/
│   └── pub_sub_demo            # Required for ROS package registration
└── README.md                   # You're here
```

---

## Build Instructions

From the root of your workspace (`~/ROB-ROS2`):

1. Source your ROS 2 environment (Jazzy):
```bash
source /opt/ros/jazzy/setup.bash
```

2. Build the workspace:
```bash
colcon build
```

3. Source the workspace overlay:
```bash
source install/setup.bash
```

---

## Run Instructions

### Option 1: Run each node individually

In separate terminals **(make sure to source `install/setup.bash` in each)**:

Terminal 1:
```bash
ros2 run pub_sub_demo talker
```

Terminal 2:
```bash
ros2 run pub_sub_demo listener
```
> Reminder: source `install/setup.bash` in both terminals

### Option 2: Use the launch file to run both nodes together

```bash
ros2 launch pub_sub_demo pub_sub_demo.launch.py
```

You should see the talker publishing messages and the listener receiving them.

---

## Using This as a Template

This package is a good starting point for creating new ROS 2 Python packages.

To create your own:

1. From inside the `src/` folder:
```bash
ros2 pkg create --build-type ament_python your_new_package
```

2. Add Python files inside:
```
your_new_package/your_new_package/*.py
```

3. Define a `main()` function in each script.

4. Register the nodes in `setup.py` under `entry_points`.

5. Add a `resource/your_new_package` file.

6. (Optional) Add a `launch/` directory and install launch files through `setup.py`.

7. Build and source the workspace:
```bash
colcon build
source install/setup.bash
```

8. Run using `ros2 run` or `ros2 launch`.

---

