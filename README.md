# UR5 Simulation Repository

This repository provides **auxiliary resources** to help students and robotics enthusiasts visualize simulations and create new nodes for their projects. It complements the [**pla10/ros2_ur5_interface**](https://hub.docker.com/r/pla10/ros2_ur5_interface) Docker image, which delivers a pre-configured ROS 2 Humble environment tailored for the **UR5 manipulator robot**.
This repo is included in the home/ros2_ws of the [**pla10/ros2_ur5_interface**](https://hub.docker.com/r/pla10/ros2_ur5_interface) Docker image.

The resources in this repository were developed for the **"Fundamentals of Robotics" course** at the **University of Trento** and aim to streamline project development and learning.

---

## Features
- **Simulation visualization**: Tools to interact with the UR5 simulation in a graphical environment.
- **Example ROS 2 nodes**: Includes a sample node for trajectory publication to get you started.
- **Launch files**: Ready-to-use launch files for interacting with the simulated or real UR5 robot.
- **Bash scripts**: Simplifies running the UR5 simulation and ROS 2 environments with pre-defined commands.

---

## Repository Structure
```plaintext
.
├── launch/
│   ├── sim.launch.py                     # Launch file to interact with simulated and real UR5 robot
├── src/
│   ├── publish_trajectory_node.cpp       # Example node for trajectory publication
├── models/
│   ├── model.urdf                        # URDF file that defines the desk where the UR5 is mounted
│   ├── model.sdf                         # SDF file that defines the desk where the UR5 is mounted
│   ├── model.config                      # Config file auxiliary to the SDF file
│   ├── mesh
│   |   ├── desk.stl                      # STL file with the 3D mesh of the desk
├── rviz/
│   ├── ur5.rviz                          # RViz configuration file
├── bash_scripts/
│   ├── ur5.sh                         # Starts the URSim simulator container
│   ├── ros2.sh                           # Starts the pla10/ros2_ur5_interface container
├── README.md                             # Project overview and instructions
```

---

## Prerequisites
- **Docker**: Ensure Docker is installed and running on your system.
- **Docker Network Setup**: Create a Docker network named `ursim_net` before running the containers:
  ```bash
  docker network create --subnet=192.168.56.0/24 ursim_net
  ```


## How to Use
### 1. Start the UR5 Simulator
Run the URSim container using the provided bash script:
```bash
bash bash_scripts/ur5.sh
```
This starts the [pla10/ursim_e-series](https://hub.docker.com/r/pla10/ursim_e-series) Docker container for UR5 simulation. Access the simulator via your browser at [http://localhost:6080](http://localhost:6080).

<img src="https://gyazo.com/7e2514442ef1753eb8b20e2b674056fc/raw" alt="UR5 Simulation" width="800">

To prepare the robot to work with ROS2 you need to go to the program tab and add the URCaps > External Control.

---

### 2. Start the ROS 2 Simulation Environment
Run the ROS 2 container using the provided bash script:
```bash
bash bash_scripts/ros2.sh
```
This starts the [pla10/ros2_ur5_interface](https://hub.docker.com/r/pla10/ros2_ur5_interface) container. Access the environment via noVNC at [http://localhost:6081](http://localhost:6081).

<img src="https://gyazo.com/ca4a65bce9e2ac0e5217edfc423d5fa9/raw" alt="UR5 Simulation" width="800">

---

### 3. Run ROS 2 Nodes
- Open a terminal inside the ROS 2 container (accessible via noVNC).
- Navigate to the ROS 2 workspace:
  ```bash
  cd home/ros2_ws
  ```
- Source the ROS 2 setup:
  ```bash
  source install/setup.bash
  ```
- Launch the simulation or control nodes using the provided launch files, e.g.:
  ```bash
  ros2 launch launch/ros2_ur5_interface.launch.py
  ```

---

## Contributions
Students and contributors are welcome to improve this repository by adding new nodes, launch files, or other auxiliary tools. Feel free to open issues or submit pull requests.

---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
