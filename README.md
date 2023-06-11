# ur_rtde

[ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html) 

## Installation

由于若想在 conda 环境中使用 tf2，需要在 conda 环境中从头编译 pykdl ，较为麻烦。因此，我们使用系统默认的 python3 来运行 ur_rtde_ros_pkg 下的程序。

因此，需要在系统默认的 python3 中安装如下包：
```
pip3 install scipy
pip3 install --user ur_rtde
```



## Test

### rostopic发布速度指令进行测试

关节速度控制：
```
rostopic pub -r 10 /control/arm/joint_vel_command ur_rtde_msgs/VectorStamped "{header: auto, data: [0, 0, 0, 0, 0, 0]}" 
```

末端速度控制：
```
rostopic pub -r 10 /control/arm/tcp_vel_in_world_command geometry_msgs/TwistStamped "{header: {frame_id: "world"}, twist: {linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}}"
```

