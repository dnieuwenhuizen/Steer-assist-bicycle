# Steer-assist-bicycle
In this repository the libraries for the steer assist bicycle and Magneto1.2 for the calibration of the IMU can be found.

Main_program falls under confidentiality but the code can be found in the appendix in my report. 

The main_program makes use of several libraries: MPU9250.h, Encoder.h, MadgwickAHRS.h, SdFat.h, Filters.h and math.h 
make sure these libraries are included when you are running in Arduino. The libraries can be downloaded direclty from this repository
and should be put in the libraries folder of the Arduino IDE. See the exmamples of the libraries to see how they work. 

The motor drive configuration is done using CME 2 software. Which can be downloaded at: http://www.copleycontrols.com/Motion/Downloads/
