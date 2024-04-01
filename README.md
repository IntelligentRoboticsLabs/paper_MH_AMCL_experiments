# paper_MH_AMCL_experiments

## Prepare repo

1. Clone
```
mkdir -p <workspace>/src
cd <workspace>/src
git clone https://github.com/IntelligentRoboticsLabs/paper_MH_AMCL_experiments.git
```
2. Prepare rosbags files:

These rosbags include experiments conducted in simulation with the Summit XL and in the real scenario with the Tiago and the MOCAP system:
```
cd <workspace>/src/paper_MH_AMCL_experiments/rosbags/
cat rosbags.zip_* > rosbags.zip
unzip rosbags.zip
```
However, due to the considerable size of the files associated with the experiments conducted outdoors with the Summit XL, they have been stored in a shared folder. To access this folder and obtain the relevant files, you can do so at this link: [Summit XL outdoor](https://urjc-my.sharepoint.com/:f:/r/personal/alberto_gomezjacinto_urjc_es/Documents/0.Research/5.%20Summit/1.%20Rosbags?csf=1&web=1&e=DWcQFO)

** To avoid potential file corruption due to the size of the rosbags, it is recommended to download the .yaml and .db3 files from each folder independently.

3. Compile and source
```
cd <workspace>
rosdep install --from-paths src --ignore-src -r
colcon build --symlink-install
source <workspace>/install/setup.bash
```

## Usage:

1. Launch simulator:

```
ros2 launch summit_xl_gazebo default.launch.py 
```

2. or play a rosbag


- Launch mh-amcl for Summit (simulated rosbags)
```
ros2 launch mh_amcl summit_launch.py 
```
- or launch mh-amcl for Tiago (real rosbags)
```
ros2 launch mh_amcl tiago_real_exp_mh3d_launch.py use_sim_time:=false
```
- Run any rosbag (simulated for Summit and real for Tiago or Summit outdoors)
```
ros2 bag play src/paper_MH_AMCL_experiments/rosbags/simulation/uneven_world_all_0/
```
3. Visualization
```
rviz2 -d src/paper_MH_AMCL_experiments/rviz_exp.rviz
```
