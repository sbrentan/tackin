# Tackin
Robotic 2021/2022 Project for Fondamenti di Robotica class by Simone Brentan, Giovanni Lanaro, Matteo Costalonga and Alex Reichert.
## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install all the required packages via the requirements.txt file using the command:
```bash
git clone https://github.com/sbrentan/tackin  # clone
cd tackin
pip install -r requirements.txt  # install
```
## Usage
Before starting make sure to run "catkin build" and "devel/setup.bash" in your environment
```bash

#launch to start a Gazebo simulation with an empty world with the grasper and set up controllers for the joints
roslaunch tackin_gazebo grasper.launch 

#starting the interactive console for the start of the assignments
rosrun tackin_control grapser.py [param] 

```
where ```[param]``` is the assignment you want to execute as : 

1. ```a1``` 

2. ```a2```

3. ```a3 [object_number]```

where ```[object_number]``` is the number of object that will spawn (≤15)

We suggest to restart the Gazebo process after every run, as it often causes simulation errors with the meshes of the spawned objects.

## License
[MIT](https://choosealicense.com/licenses/mit/)
