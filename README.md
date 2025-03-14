# digit_ros
Wrapper ROS for the DIGIT tactile sensor.

## Setting up Your Environment 

Create a ROS workspace (if you already have one, you can skip this step):  

```bash
mkdir -p ~/digit_ws/src
cd ~/digit_ws
catkin init
```

Then:  

```bash
cd ~/digit_ws/src
git clone https://github.com/FabPrez/digit_ros.git
mkdir rosinstall
cp digit_ros/digit_ros.repos rosinstall/digit_ros.repos
cd digit_ros/scripts
vcs import < ~/digit_ws/src/rosinstall/digit_ros.repos
cd ~/digit_ws
vcs pull src
sudo apt update && sudo apt upgrade -y
rosdep install --from-paths src --ignore-src -r -y
catkin config -j $(nproc --ignore=2)
catkin build -cs --mem-limit 50%
source devel/setup.bash
```

Create the Virtual Environment and Installing Dependencies

```bash
cd src/digit_ros/scripts
python3 -m venv digit_venv
source digit_venv/bin/activate
cd digit-depth
pip install -r requirements.txt
pip install .
```

### Handling Installation Failures
If the installation fails due to issues with `wheel`, `setuptools`, or `pip`, upgrade them first and then retry:

```bash
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
pip install .
```

---

## How To

### RGB Images 

```bash
roslaunch digit_ros digit_depth.launch
```

You can change your sensor ID using arguments. The default is `D20928`:  

```bash
roslaunch digit_ros digit_depth.launch id_sensor:=D20928
```
