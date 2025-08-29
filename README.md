# ros_gz_project_template
A fork of the template project integrating ROS 2, the Gazebo simulator and Golang. This also shows how to use Nix with ROS2.

Note that the selected rclgo library only works with Jazzy, however different forks of the library can be used for other ROS2 releases.

## Included packages

* `ros_gz_go_application` - holds a ROS2 node written in Go

* `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_example_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_example_application` - holds ros2 specific code and configurations.

* `ros_gz_example_bringup` - holds launch files and high level utilities.


## Install

The project contains Nix files created with `ros2nix` (i.e. `ros2nix --distro jazzy --flake $(fd package.xml)`). If you use Nix then you don't need `rosdep` and instead use `nix develop` or `nix-shell`.

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation

   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

   If you need something other than jazzy, then see the note at the top about rclgo.

    ```bash
    export GZ_VERSION=jazzy
    ```
    Also need to build [`ros_gz`](https://github.com/gazebosim/ros_gz) and [`sdformat_urdf`](https://github.com/ros/sdformat_urdf) from source if binaries are not available for your chosen combination.

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

### Use as template
Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

   ```bash
   mkdir -p ~/template_ws/src
   cd ~/template_ws/src
   git clone https://github.com/gazebosim/ros_gz_project_template.git
   ```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
    ```
    
    or if using Nix

    ```bash
    NIXPKGS_ALLOW_INSECURE=1 nix develop --impure
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_example_bringup diff_drive.launch.py
    ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
