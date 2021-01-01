# Spaceryde_Design_Challenge
My solution to the software design challenge for the software engineering intern position at Spaceryde.

# Usage
Make sure to clone recursively with `git clone --recursive https://github.com/RealWilliamWells/Spaceryde_Design_Challenge`, in order to clone both the main repository and the necessary submodules used in the catkin workspace.

In the terminal, cd to catkin_ws. Then run the following commands in order to compile everything: 

`rosdep install --from-paths src --ignore-src -r -y`

`catkin_make`

Source the workspace, then run `rosrun dynamics_simulator dynamics_simulator` to run the real time simulation with the sensor simulation. The current Kalman filter code is still WIP, so it currently isn't ran with the rest of the simulation.
