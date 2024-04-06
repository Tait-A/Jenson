# Jenson

A Dissertation Project for racing robots

This repository contains the majority of the code for this dissertation.
This repository is for the work station/laptop side of the system,
the counterpart to this is the Nigel repository, which contains the code for the Raspberry Pi.

In this repository:

The car folder contains the source .stl files used to 3D print the car.
A few modifications were made to some of these files in subsequent versions, some of which are included.

The src folder contains the code for this repository and other source files.

In the src folder:

Within the src folder is the config file used for various configuration constants, and the init file to make a module.
The test.py file is no longer used. The requirements.txt file can be used to configure a python environment for this repository.

The Control Folder contains the Model Predictive Controller implementation.
Running this file will perform some control optimisations and generate a control figure.
There is currently an issue where the control cannot cope with passing from -pi to pi angles, and performs loops,
caused by an issue in the solver. A new solver library or a custom implementation is needed.

The JSON folder is not contained in this zip, but JSON output of various files will be generated here.
It principally contains JSON representations of trajectories.
These JSON representations would have been used to send to the MQTT topic.

The Localisation folder is not in use, the localisation implementation can be found in python notebooks in the
Testing folder and will be discussed shortly.

The Models folder contains the Planning and Parameterisation Implementations, as well as the Robot class, which primarily
contains definitions of various robot parameters. Running the Track file will load the track waypoints from the track.json
file which has been moved from the JSON folder for your convenience. The midline is found and optimised and a figure is generated.
This makes use of the QES and input_profile files to optimise the path and generate the parameterisations.

The Output folder has similarly not been included in this zip due to it's size. It contains the images taken from the pi
used to localise the trajectories. This cannot be included due to the almost 1000 images contained in it, however a sample of
images can be provided if required, or any consecutive series of images should work as expected.

The Testing folder contains implementations of feature extraction and localisation in the featureExtraction.ipynb file.
The images required to run this file are not included due to size, however the notebook file should have saved the state of
the outputs of the cells so the various visualisations can be seen.
The image_processing file contains the code used to generate the image processing visualisations seen in the appendix of the dissertation.
The m2bk.py file contains the image dataset handling code. This was taken from course code for the
Introduction to Mobile Robotics course, with slight modifications made for application here.
The other python notebooks contain various tests and visualisations for different aspects of the project.

The Trajectories folder contains test trajectories used on the robot by the Nigel Repository.

The Utils folder contains lots of utility code written by the author, for calibrating the camera,
for receiving images from the Pi, and the pi code for sending them.
Most importantly it contains the Spline, State, Action and Trajectory Classes, used throughout this dissertaion
for the manipulation and processing of imformation. The Splines class is especially important to the optimisation
of the path as a specialised wrapper for spline objects from the CSAPS library.
