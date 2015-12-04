# ROSMAT: ROlling Spider MAtlab Toolbox

This toolbox provides an environment to enhance the learning experience of design, simulation and testing of estimation and control modules for a small palm-sized drone, the Parrot Rolling Spider. A MATLAB/Simulink model allows to design and simulate the algorithms, the automatic generation of c-code directly from Simulink makes it easy to push the designed estimation- and control-algorithms onto the drone and test its behavior in real-life.

The first release of this toolbox was developed by Fabian Riether and Prof. Sertac Karaman to enhance "16.30 Feedback Control Systems", a course offered at the Department of Aeronautics and Astronautics at the Massachusetts Institute of Technology (MIT). More information on the course and relevant course material can be found at:
http://karaman.mit.edu/1630/


Please keep in mind that this toolbox is for educational purposes and is therefore rather tuned to be easily understood than to meet software engineering standards and amazing flight performance. The first version is mainly designed for experimenting with hover flight. We encourage you to make use of GitHub's issue tracker to submit issues and contribute with your own cool stuff!

The drone's dynamics are simulated using code of Peter Corke's Robotics Toolbox http://www.petercorke.com/Robotics_Toolbox.html.

To make this toolbox possible, Parrot SA provided a compiler toolchain and a custom firmware that opens up three interfaces to insert custom controls code, image processing code and optical flow handling.

### Installing and Getting Started ...
To install the toolbox, get familiar with the work flow and hit the ground running,
please read https://github.com/Parrot-Developers/RollingSpiderEdu/blob/master/MIT_MatlabToolbox/media/GettingStarted.pdf
