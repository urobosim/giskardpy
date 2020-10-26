# giskardpy
The core python library of the Giskard framework for constraint- and optimization-based robot motion control.

## pybullet:
Kineverse is a new lib used by Giskard. It uses a custom pybullet wrapper. 
Install as described here: https://github.com/ARoefer/kineverse/tree/giskard_merge#installing-bullet
Ignore the kineverse installation!
It is part of the rosinstall below.

## Installation instructions. Tested with Ubuntu 16.04 + ROS kinetic and 18.04 + melodic

Install the following python packages:
```
sudo pip install pybullet
sudo pip install scipy==1.2.2 # this is the last version for python 2.7
sudo pip install casadi
sudo pip install sortedcontainers
sudo pip install hypothesis==4.34.0 # only needed if you want to run tests
sudo pip install pandas==0.24.2
sudo pip install numpy==1.16.6
sudo pip install matplotlib<=2.2.5
sudo pip install simplejson 
sudo pip install tqdm
sudo pip install jinja2
```

Now create the workspace
```
source /opt/ros/kinetic/setup.bash          # start using ROS kinetic. Replace with melodic, if you are using it.
mkdir -p ~/giskardpy_ws/src                 # create directory for workspace
cd ~/giskardpy_ws                           # go to workspace directory
catkin init                                 # init workspace, you might have to pip install catkin-tools
cd src                                      # go to source directory of workspace
wstool init                                 # init rosinstall
wstool merge https://raw.githubusercontent.com/SemRoCo/giskardpy/master/rosinstall/catkin.rosinstall
                                            # update rosinstall file
wstool update                               # pull source repositories
rosdep install --ignore-src --from-paths .  # install dependencies available through apt
cd ..                                       # go to workspace directory
catkin build                                # build packages
source ~/giskardpy_ws/devel/setup.bash      # source new overlay
```

### Tutorials
http://giskard.de/wiki:tutorials

