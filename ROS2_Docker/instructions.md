# Docker
**Build the Docker Image:**
1. run docker engine in docker desktop
2. Open a terminal, cd to the directory containing your Dockerfile and run these 2 commands:
<br>

**build the Docker Container:**

``
docker build -t ros2_image .
``

**Run the Docker Container:**

``
docker run -it ros2_image
``

*This setup creates a Docker image based on Ubuntu, installs any necessary dependencies, and runs your ROS 2 installer script inside the container.*
<br>

**type "exit" to leave the container in your terminal**
<br>
