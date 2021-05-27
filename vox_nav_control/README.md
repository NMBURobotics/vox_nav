Refer to bringup/params.yaml for configuring controller server. 

```yaml
vox_nav_controller_server_rclcpp_node:
  ros__parameters:
    controller_plugin: "MPCControllerROS" # other options: "MPCControllerROS"
    controller_frequency: 15.0
    MPCControllerROS:
      plugin: "mpc_controller::MPCControllerROS"
      N: 8                                                        #timesteps in MPC Horizon
      DT: 0.2                                                     #discretization time between timesteps(s)    
      L_F: 0.66                                                   #distance from CoG to front axle(m)
      L_R: 0.66                                                   #distance from CoG to rear axle(m)
      V_MIN: -1.0                                                 #min / max velocity constraint(m / s)
      V_MAX:  1.0
      A_MIN:  -1.0                                                #min / max acceleration constraint(m / s ^ 2)
      A_MAX:   1.0
      DF_MIN:  -1.5                                               #min / max front steer angle constraint(rad)
      DF_MAX:   1.5
      A_DOT_MIN: -1.0                                             #min / max jerk constraint(m / s ^ 3)
      A_DOT_MAX:  1.0
      DF_DOT_MIN: -0.8                                            #min / max front steer angle rate constraint(rad / s)
      DF_DOT_MAX: 0.8
      Q: [10.0, 10.0, 0.1, 0.1]                                  #weights on x, y, psi, and v.
      R: [10.0, 100.0]                                            #weights on jerk and slew rate(steering angle derivative)
      debug_mode: False                                           #enable/disable debug messages
      params_configured: False      
   
```

Select a plugin out of 1 available plugins listed as ; MPCControllerROS. 

Test the controller by sending an all zeros path with ;
 
```bash
ros2 action send_goal /follow_path vox_nav_msgs/action/FollowPath "{}"
``` 