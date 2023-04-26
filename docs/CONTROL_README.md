## Control server

This page introduces a more in-depth overview to controller server and plugins.


This server brings up a plugin of your choice live. The controller server is implemnted with ROS2 actions. 
Following parameters are configurable;


Available controller plugins; 
* `MPCControllerCasadiROS`
* `MPCControllerAcadoROS`
* `LyapunocController`

### 1. `MPCControllerCasadiROS` Plugin

Implemented with casadi non-linear optimization framework. 

YAML parameters for this plugin;

```yaml
MPCControllerCasadiROS:
   plugin: "mpc_controller_casadi::MPCControllerCasadiROS"
   N: 8                                                                 # timesteps in MPC Horizon
   DT: 0.1                                                              # discretization time between timesteps(s)
   L_F: 0.66                                                            # distance from CoG to front axle(m)
   L_R: 0.66                                                            # distance from CoG to rear axle(m)
   V_MIN: -1.5                                                          # min / max velocity constraint(m / s)
   V_MAX: 1.5
   A_MIN: -1.5                                                          # min / max acceleration constraint(m / s ^ 2)
   A_MAX: 1.5
   DF_MIN: -0.4                                                         # min / max front steer angle constraint(rad)
   DF_MAX: 0.4
   A_DOT_MIN: -1.0                                                      # min / max jerk constraint(m / s ^ 3)
   A_DOT_MAX: 1.0
   DF_DOT_MIN: -0.8                                                     # min / max front steer angle rate constraint(rad / s)
   DF_DOT_MAX: 0.8
   Q: [10.0, 10.0, 0.1, 0.1]                                            # weights on x, y, psi, and v.
   R: [10.0, 100.0]                                                     # weights on jerk and slew rate(steering angle derivative)
   debug_mode: False                                                    # enable/disable debug messages
```
### 2. `MPCControllerAcadoROS` Plugin

If there has not been already, you need to generate the Acado code with; 

```bash
ros2 launch vox_nav_control acado_code_generation.launch.py
```

Pleaase NOTE that the generated code for Acado must be under `vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen`

Before executing above command, make sure to configure the parameters of `MPCControllerAcadoROS`, the code will be generated according to selected parameters.

If you delete `vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen` for some reason, 
First, only build acado_code_generation with; 

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=OFF
```
This will only build code generation node, then execute `acado_code_generation.launch.py` to generate the code. 

```bash
ros2 launch vox_nav_control acado_code_generation.launch.py
```
Then place the generated codes under `vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen`. 

Finally build the `MPCControllerAcadoROS` with `-DACADO_CODE_IS_READY=ON`;

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON
```

YAML parameters for this plugin;

```yaml
MPCControllerAcadoROS:
   plugin: "mpc_controller_acado::MPCControllerAcadoROS"
   N: 20                                                                # timesteps in MPC Horizon
   Ni: 1
   DT: 0.1                                                              # discretization time between timesteps(s)
   L_F: 0.67                                                            # distance from CoG to front axle(m)
   L_R: 0.67                                                            # distance from CoG to rear axle(m)
   V_MIN: -1.0                                                          # min / max velocity constraint(m / s)
   V_MAX: 1.0
   A_MIN: -1.0                                                          # min / max acceleration constraint(m / s ^ 2)
   A_MAX: 1.0
   DF_MIN: -0.6                                                         # min / max front steer angle constraint(rad)
   DF_MAX: 0.6
   Q: [10.0, 10.0, 0.0, 0.0]                                            # weights on x, y, v, and psi.
   R: [10.0, 100.0]                                                     # weights on input acc and df, acceleration and steering angle
   debug_mode: False                                                    # enable/disable debug messages
```

### 3. `LyapunocController` Plugin
A controller based on [Control Lyapunov Function](https://arxiv.org/pdf/2210.02837v1.pdf). Details on this controller are linked in the related publication. 

YAML parameters for this plugin;
```yaml
LyapunovControllerROS:
   plugin: "lyapunov_controller::LyapunovControllerROS"
   V_MIN: -0.5                                                          # min / max velocity constraint(m / s)
   V_MAX: 0.5
   DF_MIN: -0.125                                                       # min / max front steer angle constraint(rad)
   DF_MAX: 0.125
   k1: -1.0                                                             # Control coefficents, see the related paper. 
   k2: 5.0
   lookahead_n_waypoints: 4                                             # Number of waypoints on the path to follow
```

### 4. Test
Test the controller server by sending an all zeros path with ;
 
```bash
ros2 action send_goal /follow_path vox_nav_msgs/action/FollowPath "{}"
``` 