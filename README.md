# pararl
otter project


# test demo 
1. test continuous control mode
To test kinova env, run `python test/keyboard_control.py` and use keyboard to control the end-effector in Cartesian space.

2. test discrete control mode 
run `python test/test_discrete_kinova_control.py' and use keyboard(1,2,3,4,5,6) to control ee to move.


# steps for setting ros enviroment
1. export PATH=~/anaconda3/bin:$PATH
2. source activate py35
3. source ~/ProjectOtter/pararl2/ROS/devel/setup.bash
4. ./pycharm-2018.3.3/bin/pycharm.sh

# steps to set up real robot
open a terminal, then input
1. export PYTHONPATH="/usr/lib/python2.7/dist-packages:$PYTHONPATH"
2. roslaunch kinova_bringup kinova_robot.launch

open another terminal, then input
1. export PYTHONPATH="/usr/lib/python2.7/dist-packages:$PYTHONPATH"
2. rosrun otter_kinova_grasping kinova_control.py 

#steps to set up realsense
1. export PYTHONPATH="/usr/lib/python2.7/dist-packages:$PYTHONPATH"
2. roslaunch otter_kinova_grasping kinova_camera.launch 




