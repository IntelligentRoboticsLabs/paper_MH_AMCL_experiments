# paper_MH_AMCL_experiments

## Usage:


Launch simulator:

```
ros2 launch summit_xl_gazebo default.launch.py 
```
or play a rosbag



Launch mh-amcl

```
ros2 launch mh_amcl summit_launch.py 
```

Experiments:
```
ros2 launch mh_amcl tiago_real_exp_mh3d_launch.py use_sim_time:=false
rviz2 -d src/paper_MH_AMCL_experiments/rviz_exp.rviz
```
