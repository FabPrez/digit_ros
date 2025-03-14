# digit_ros
wrapper ros for tactile sensor digit
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
vcs import < rosinstall/digit_ros.repos
cd ~/digit_ws
vcs pull src
sudo apt update && sudo apt upgrade -y
rosdep install --from-paths src --ignore-src -r -y
catkin config -j $(nproc --ignore=2)
catkin build -cs --mem-limit 50%
source devel/setup.bash
```

Create the virtual environment and install the requirements:

```bash
cd src/digit_ros/scripts
python3 -m venv digit_venv
source digit_venv/bin/activate
pip install -r requirements.txt
pip install .
```

