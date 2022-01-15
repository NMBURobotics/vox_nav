Refer to `vox_nav/vox_nav_bringup/params/vox_nav_default_params.yaml` for configuring controller server. 

Currently Two plugins for controller are available; `MPCControllerCasadiROS` and `MPCControllerAcadoROS`.
These to are MPC implementation using differnt tools. Acado generates extremely fast C code while Casadi generates OK speed. 

You can generate the Acado code with; 

```bash
ros2 launch vox_nav_control acado_code_generation.launch.py
```

Pleaase NOTE that the generated code for Acado must be under `vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen`
If your workspace name is differnt than `colcon_ws`, you need change the full path in `vox_nav/vox_nav_control/src/mpc_controller_acado/mpc_controller_acado_code_gen.cpp`

Before executing above command, make sure to configure the parameters of `MPCControllerAcadoROS`, the code will be generated according to selected parameters.

After you generate the code you can build, If you delete `vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen` for some reason, 
First, only build acado_code_generation with; 

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=OFF
```
This will only build code generation script, then execute `acado_code_generation.launch.py` to generate the code. 
Finally build the `MPCControllerAcadoROS` with;

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON
```

```yaml
vox_nav_controller_server_rclcpp_node:
   ros__parameters:
      controller_plugin: "MPCControllerAcadoROS"                              # other options: non
      controller_frequency: 200.0                                             # acado is really fast(+1000.0Hz) casadi can deal up to just 20.0Hz maybe
      goal_tolerance_distance: 0.2                                            # in meters, once smaller than this one controller tries to minimize orientation error
      goal_tolerance_orientation: 0.1                                         # in radians, once smaller than this value,controller exits with success
      transform_timeout: 0.01                                                 # seconds, this is used to regulate lookUpTransfrom calls from tf2
      global_plan_look_ahead_distance: 3.5                                    # look this amount of meters from current robot pose to remaining global path
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
         params_configured: True
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

Select a plugin out of 2 available plugins listed as ; `MPCControllerCasadiROS`, `MPCControllerAcadoROS`

Test the controller by sending an all zeros path with ;
 
```bash
ros2 action send_goal /follow_path vox_nav_msgs/action/FollowPath "{}"
``` 