
## Start Unitree controller
```bash
sudo ./devel/lib/unitree_guide/junior_ctrl
```

## Print topic type
```bash
rostopic type /go1_gazebo/FL_calf_controller/command
unitree_legged_msgs/MotorCmd

rostopic type /go1_gazebo/FL_calf_controller/joint_wrench
geometry_msgs/WrenchStamped

rostopic type /go1_gazebo/FL_calf_controller/state
unitree_legged_msgs/MotorState
```

## Print topic content
```bash
rostopic echo /go1_gazebo/FL_calf_controller/command
```

### Initial state
```
mode: 10
q: 0.0
dq: 0.0
tau: 0.0
Kp: 0.0
Kd: 8.0
reserve: [0, 0, 0]
```

### Press 2, Switched from passive to fixed stand
```
mode: 10
q: -1.2999999523162842
dq: 0.0
tau: 0.0
Kp: 300.0
Kd: 15.0
reserve: [0, 0, 0]
```

### Press 4,Switched from fixed stand to trotting
```
mode: 10
q: -1.4883830547332764
dq: 0.00035537060466594994
tau: 4.4783525466918945
Kp: 0.800000011920929
Kd: 0.800000011920929
reserve: [0, 0, 0]
```

### Press w, keep changing values
```
mode: 10
q: -1.5103154182434082
dq: -0.14720816910266876
tau: 9.760361671447754
Kp: 0.800000011920929
Kd: 0.800000011920929
reserve: [0, 0, 0]
---
mode: 10
q: -1.5107638835906982
dq: -0.13323193788528442
tau: 9.769333839416504
Kp: 0.800000011920929
Kd: 0.800000011920929
reserve: [0, 0, 0]
---
mode: 10
q: -1.5110745429992676
dq: -0.11927668005228043
tau: 9.77442455291748
Kp: 0.800000011920929
Kd: 0.800000011920929
reserve: [0, 0, 0]
---
mode: 10
q: -1.5114425420761108
dq: -0.1049032136797905
tau: 9.776960372924805
Kp: 0.800000011920929
Kd: 0.800000011920929
reserve: [0, 0, 0]
---
mode: 10
q: -1.5118534564971924
dq: -0.08875811845064163
tau: 9.776429176330566
Kp: 0.800000011920929
Kd: 0.800000011920929
reserve: [0, 0, 0]
---
mode: 10
q: -1.512108325958252
dq: -0.07489873468875885
tau: 9.77553939819336
Kp: 0.800000011920929
Kd: 0.800000011920929
reserve: [0, 0, 0]
```

