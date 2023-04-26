
## VOX_NAV Installation;

* Install ROS2 Humble desktop. Deb installation is strongly recomended. 
You can always find updated step to install [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages)


* Install deps;

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install -y python3-rosdep2
sudo apt-get install python3-vcstool
sudo apt-get install xdotool
sudo apt-get install coinor-libipopt-dev
sudo apt install -y libfcl0.7
sudo apt install -y pkg-config
sudo apt install -y libmosquitto*
```

* Create a ROS2 workspace and get dependencies that require source build;

```bash
mkdir -p ~/ros2_ws/src
source /opt/ros/humble/setup.bash
rosdep update
cd ~/ros2_ws
wget https://raw.githubusercontent.com/jediofgever/vox_nav/humble/underlay.repos
vcs import src < underlay.repos --recursive   
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble --skip-keys="cartographer-ros cartographer_ros"
```
* Now build ompl and casadi only;
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON -DWITH_IPOPT=true --packages-select ompl casadi; \
sudo cp install/ompl/lib/libompl.so* /usr/local/lib/
sudo cp install/casadi/lib/libcasadi.so* /usr/local/lib/ 
source install/setup.bash  
```

* Build vox_nav packages except for `vox_nav_control vox_nav_misc`

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON -DWITH_IPOPT=true --packages-skip-regex archive --packages-skip vox_nav_control vox_nav_misc; \
source /opt/ros/humble/setup.bash
```

* Build vox_nav control as the ACADO has been built already.
```bash 
cd ~/ros2_ws
source build/ACADO/acado_env.sh
source install/setup.bash 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON -DWITH_IPOPT=true --packages-select vox_nav_control; \
``` 

> **_NOTE:_**  If using [lidar_apollo_instance_segmentation](https://github.com/jediofgever/lidar_apollo_instance_segmentation), build while point to `TENSORRT_ROOT`.

```bash 
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release -DACADOS_WITH_QPOASES=on -DACADO_CODE_IS_READY=on -DWITH_IPOPT=true -DTENSORRT_ROOT=/home/atas/downloads/tensorrt-8.4.1.5 -DHUMBLE_ROS=humble --packages-skip-regex archive --packages-skip vox_nav_control
```


There are dependency libraries that needs source build e.g., OMPL and casadi etc..
The above coammmdns will build and install them.

> **_NOTE:_** Pay attention that we have disabled the build of archived_ packages and vox_nav_misc. The archived package are not in use and not needed but are kept for convenience.


> **_NOTE:_**  If for some reason you could not build vox_nav, a good place to seek for a solution is the github actions file that we have.
After each pull-push github actions is setup to build the vox_nav on remote to ensure stability of builds. Find a recent successful build and see the commands in .github/workflows/main.yml. The commands should more or less look as in this page.
