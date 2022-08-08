
# Simulated enviroment for Unmanned Surface Vehicles (usv_sim)

Forked from https://github.com/disaster-robotics-proalertas/usv_sim_lsa.

## Prerequisites

You need to have Ubuntu 20.04 LTS, ROS Noetic, and Gazebo 11 installed and set up with a catkin workspace before starting. Also install Open Scene Graphics version 3.2.3.

Now run the following commands to download the dependencies of usv_sim:

        sudo apt-get install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep python-wxtools python3-lxml python3-h5py python3-scipy python3-geolinks python3-gdal -y
        sudo apt-get install libfftw3-* libxml++2.6-* libsdl-image1.2-dev libsdl-dev -y


## Installing

Clone the usv_sim repository in the src folder of your catkin workspace:

        cd ~/catkin_ws/src
        git clone https://github.com/courtneymcbeth/usv_sim.git
        cd usv_sim
        git submodule init
        git submodule update
        git submodule foreach git pull origin master-melodic

The last command above might give you errors. You can safely ignore them.
Run the instalation script:

        cd ~/catkin_ws/src/usv_sim
        chmod +x ./install_usv_sim
        ./install_usv_sim

Compile the stack:

        cd ~/catkin_ws/
        catkin_make_isolated --install -j1
        source install_isolated/setup.bash
        chmod +x ~/catkin_ws/src/usv_sim/uwsim_resources/underwater_simulation/uwsim/src/uwsim

To run a scenario:

        roslaunch usv_sim scenario_name.launch parse:=true
        roslaunch usv_sim scenario_name.launch parse:=false

The simulation might take some time to initialize if you're launching gazebo for the first time. If the simulation dosen't starts you should close it, run gazebo separately (command *gazebo* in the terminal), wait for gazebo to open (it is downloading some models), close gazebo and then try to run the scenario again.

Make sure your graphic card driver is up to date.