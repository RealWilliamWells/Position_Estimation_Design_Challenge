# Position_Estimation_Design_Challenge
My solution to the software design challenge for the software engineering intern position at an unnamed company. Coded almost entirely in C++.

# Submodules used
graph_rviz_plugin
https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin.git
- For real time graphing

eigen
https://gitlab.com/libeigen/eigen.git
- For matrix data type and operations

# Usage
Make sure to clone recursively with `git clone --recursive https://github.com/RealWilliamWells/Spaceryde_Design_Challenge`, in order to clone both the main repository and the necessary submodules used in the catkin workspace. Not necessary if sent in zip file with submodules included.

In the terminal, goto the "catkin_ws" folder. Then run the following commands in order to compile everything: 

`rosdep install --from-paths src --ignore-src -r -y`

`catkin_make`

Source the workspace, then run `rosrun dynamics_simulator run_simulation_graph_gps_estimation.launch` to run the real time simulation with the sensor simulation. With real time graphing for the true position, position according to the GPS sensor and the estimated position.

Or, run `run_simulation_graph_accelerometer.launch` to run the real time simulation but with real time graphing for the accelerometer readings only.

*NOTE:* The real time graphing has to be manually started, the simulation gives you a count down from 10 to start the real time graphing in the rViz window opened.

# Documentation
In the "documentation" folder, photos of the real time graphing and the pre-plotted system's dynamics for step two can be found. With the python script used for plotting step two. Along with a pdf file outlining all of my calculations done for the challenge. 
