# wheel-stuck-ros-pkgs

Fool Stuck Robot

## Requirements

- Ubuntu 22.04
- ROS 2 Humble

## 1. Setup

1. Update pkg list

   ```bash
   sudo apt update
   ```

2. Install VCS tool and rosdep

   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep
   sudo apt install -y python3-vcstool
   ```

3. Clone repos and cd into dir

   ```bash
   git clone https://github.com/Fool-Stuck/wheel-stuck-ros-pkgs.git && cd wheel-stuck-ros-pkgs
   ```

4. Import depend pkgs(source)

   ```bash
   vcs import src < depend_packages.repos --recursive
   ```

5. Setup rosdep

   ```bash
   sudo rosdep init && rosdep update
   ```

6. Install depend pkgs(binary)

   ```bash
   rosdep install -i -y --from-paths src --ignore-src
   ```

7. Build

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## 2. Run

## LICENSE
