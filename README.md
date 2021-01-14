# Capstone
Capstone Project

This code is written for McMaster Mechtronics Capstone Project.

The code using following librarlies
- Basic C and C++ libraries
- OpenMP
- openCV Version 3.3.1

The code is operating under the following operating system
- Window
- Linux

Because the code was needed to be used with a low-performance hardware, the code excluded a function or method that requires high-performance hardware as possible.

About the code.
This code is specially written for indentifying buoy lane of rowing race.
This code receives real-time image data which includes the buoy lane from a connected camera and a video file.

The code performs image processing to extract position data of buoys in the image data.
After the data is extracted, it applies Robust Linear Regression method to calculate required data for buoy line dwaring.
Also, with the buoy line data, the system calculates a suggesting boat trajectory.

After the entire required is prepared, the software modifies  original image data to display the buoy line and the suggesting boat directory.
