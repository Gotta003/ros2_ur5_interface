Here’s an updated version of the **README** reflecting your updated purpose:

```markdown
# UR5 Simulation Repository

This repository provides **auxiliary resources** to help students and robotics enthusiasts visualize simulations and create new nodes for their course projects. It complements the **pla10/ur5_simulation** Docker image, which delivers a pre-configured ROS 2 Humble environment tailored for the **UR5 manipulator robot**.

The resources in this repository were developed for the **"Fundamentals of Robotics" course** at the **University of Trento** and aim to streamline project development and learning.

<img src="https://gyazo.com/ca4a65bce9e2ac0e5217edfc423d5fa9/raw" alt="UR5 Simulation" width="800">

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
│   ├── ur5_simulation.launch.py   # Launch file to interact with simulated and real UR5 robot
├── src/
│   ├── trajectory_node.cpp        # Example node for trajectory publication
├── bash_scripts/
│   ├── run_ursim.sh               # Starts the URSim simulator container
│   ├── run_ros2_sim.sh            # Starts the pla10/ur5_simulation container
├── README.md                      # Project overview and instructions
```

---

## Prerequisites
- **Docker**: Ensure Docker is installed and running on your system.
- **Docker Network Setup**: Create a Docker network named `ursim_net` before running the containers:
  ```bash
  docker network create --subnet=192.168.56.0/24 ursim_net
  ```

---

## How to Use
### 1. Start the UR5 Simulator
Run the URSim container using the provided bash script:
```bash
bash bash_scripts/run_ursim.sh
```
This starts the [pla10/ursim_e-series](https://hub.docker.com/r/pla10/ursim_e-series) Docker container for UR5 simulation. Access the simulator via your browser at [http://localhost:6080](http://localhost:6080).

---

### 2. Start the ROS 2 Simulation Environment
Run the ROS 2 container using the provided bash script:
```bash
bash bash_scripts/run_ros2_sim.sh
```
This starts the [pla10/ur5_simulation](https://hub.docker.com/r/pla10/ur5_simulation) container. Access the environment via noVNC at [http://localhost:6081](http://localhost:6081).

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
  ros2 launch launch/ur5_simulation.launch.py
  ```

---

## Contributions
Students and contributors are welcome to improve this repository by adding new nodes, launch files, or other auxiliary tools. Feel free to open issues or submit pull requests.

---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
```

### Adjustments:
- Changed focus to providing **auxiliary resources** for the course and project development.
- Highlighted how the repository supports simulation visualization and node creation.
- Updated the contributions section to encourage students to expand the repository's content. 

Let me know if more adjustments are needed!
