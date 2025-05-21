# README: ROS 2 Workspace Instructions

This repository contains a ROS 2 workspace designed for controlling a robot using a motion controller and a bringup package. You don't need to download or install any dependencies manually. Simply build the development container to get started. Alternatively, you can run the workspace through your terminal using Docker commands, which will be explained in a later section.

---

## Workspace Structure

- **`motion_controller`**: Subscribes to target positions and publishes velocity commands.
- **`target_pos_issuer`**: Publishes random 2D target positions.
- **`robot_bringup`**: Launches all nodes, including the `motion_controller` and `target_pos_issuer`.

---

# Setting Up Your Workspace

You can set up the workspace in two ways:

## Option 1: Using VS Code with Dev Containers (Recommended)

This is the easiest method if you're using Visual Studio Code.

1. **Prerequisites**:
   - Install [Visual Studio Code](https://code.visualstudio.com/)
   - Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
   - Install [Docker](https://www.docker.com/products/docker-desktop/) and ensure it's running

2. **Open in VS Code**:
   - Open the repository folder in VS Code
   - When prompted, click "Reopen in Container" or use the command palette (F1) and select "Dev Containers: Reopen in Container"
   - VS Code will build and start the container automatically (this may take a few minutes the first time)

3. **Build the Workspace**:
   Once the container is running and VS Code is connected, open a terminal in VS Code and run:
   ```bash
   colcon build
   ```

4. **Source the Workspace**:
   ```bash
   source install/setup.bash
   ```

## Option 2: Using the Terminal

Use this method if you prefer working directly with Docker commands or don't use VS Code.

1. **Prerequisites**:
   - Install [Docker](https://www.docker.com/products/docker-desktop/) and Docker Compose
   - Ensure Docker service is running

2. **Navigate to Repository Directory**:
   ```bash
   cd /path/to/repository
   ```

3. **Build and Start the Container**:
   ```bash
   docker compose build
   docker compose up -d
   ```

4. **Access the Container**:
   ```bash
   docker exec -it tuesdaylabs-ros2-dev-1 bash
   ```
   (The container name may vary based on your directory name. You can check active containers with `docker ps`)

5. **Initialize ROS Dependencies** (first time only):
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

6. **Build the Workspace**:
   ```bash
   colcon build
   ```

7. **Source the Workspace**:
   ```bash
   source install/setup.bash
   ```

8. **Stopping the Container** (when finished):
   ```bash
   # Exit the container first with 'exit' command, then:
   docker-compose down
   ```

---

## Running the Robot Bringup

The `robot_bringup` package launches all necessary nodes, including the `motion_controller` and `target_pos_issuer`. It also optionally launches the TurtleBot3 Gazebo simulation.

### Launch the Robot Bringup

1. **Run the Launch File**:
   ```bash
   ros2 launch robot_bringup robot.launch.py
   ```

2. **Optional Arguments**:
   - `headless`: Set to `true` to run without Gazebo.
     ```bash
     ros2 launch robot_bringup robot.launch.py headless:=true
     ```

---

## Running Individual Nodes

### Run the `motion_controller` Node

1. **Launch the Node**:
   ```bash
   ros2 run motion_controller motion_controller_node
   ```

### Run the `target_pos_issuer` Node

1. **Launch the Node**:
   ```bash
   ros2 run target_pos_issuer target_pos_issuer_node
   ```

---

## Testing Your Code

To add new test cases to your Python package:

1. **Create Test Files**: 
   Add new test files in the existing test directory with names starting with `test_`.
   ```bash
   # Example: creating a new test file
   touch src/your_package/test/test_your_new_feature.py
   ```

2. **Run All Tests**:
   ```bash
   colcon test --packages-select your_package
   ```

3. **View Test Results**:
   ```bash
   colcon test-result --verbose
   ```

---

## Notes

- **Gazebo Simulation**: If running with Gazebo, ensure you have the TurtleBot3 Gazebo packages installed.
- **Environment Variables**: The Docker container automatically sets up the ROS environment. If running outside Docker, source the ROS 2 and workspace setup files:
  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ```

---
