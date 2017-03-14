# Self Driving Car Comps 2017

[Our Website](http://www.cs.carleton.edu/cs_comps/1617/sgoings1/final-results2/index.html)

The files in this directory rely on an underlying installation of Gazebo
found in /Library/Caches/Homebrew/gazebo-6.1.0

Logic that controls the car can be found in the following files:
From 2016:
  - sdcAngle.cc - provides a custom angle class that wraps from 0 to 2Pi
  - sdcLidarRay.cc - wraps a single ray of a Lidar sensor into an angle and distance relative to the car and provides some convenience methods
  - sdcSensorData.cc - holds available data from sensors, as well as convenience methods; provides the bridge between sdcCar and all sensor plugins
  - sdcVisibleObject.cc - wraps obstructions detected in the Lidar sensors into trackable objects with several convenience and logical methods

Modified/Added in 2017:
  - sdcCar.cc - the main class for the car, controls logical decisions made based on all available data and instructions. It contains code for driving using a camera sensor to drive a car along curved and straight roads, passing cars in its lane using lidar, and navigating intersections with the help of an intersection manager.

  - sdcCameraSensor.cc - provides the backbone for our computer vision algorithms

  - manager.cc - Implements the stop sign reservation algorithm and connects sensors to their cars

  - sdcManager.cc - Implements the improved intersection reservation algorithm


Other files contain data gathered by sensors or transfer sensor data
from Gazebo's core functionality to our plugins.



## HOW TO SET UP THE PROJECT
You will need to have installed [Gazebo 6](http://gazebosim.org/download).
Gazebo is currently up to 8.0.0 but we do not guarantee that our code is compatible with any
version other than 6. We were not able to test our project on any other operating system so we do not guarantee
compatibility with linux or windows.
If you are on a mac you can install gazebo 6 using homebrew. To do so use the command:
brew install gazebo6 --HEAD
Or follow the instructions on [this page](http://gazebosim.org/tutorials?cat=install&tut=install_on_mac&ver=6.0#InstallGazeboonMac)


You will also need [CMAKE](https://cmake.org/download/) and [OpenCV](http://opencv.org/downloads.html) to run the project.

Once you have installed the necessary dependencies and have cloned the repository you will need to add the models folder to your local gazebo folder. To do so copy the models folder in the repository to your hidden gazebo folder for your user (the command open .gazebo should work).


## COMPILING THE PROJECT
To compile the project you will need to create a folder named "build" in the SDC-Merged folder.
If a build folder already exists and has files in it you should empty the folder (you only need to do this the first time you compile).
Once the build folder is created you can either run "bash compile" in console from
the SDC-Merged directory or cd into build, type "cmake ../", and then "make".

## RUNNING THE SIMULATIONS
Now you should be able to run our code! Gazebo simulations are called world plugins.
We have created various world plugins that show off the features of our project.

In order to see the driving capabilities of our car run the following world files
by using the command "gazebo worldNameHere.world"

Stop sign intersection worlds:
These worlds show off intersection management using a stop-sign-like policy.
Cars will approach the intersection, stop, and then go through the intersection
if the intersection manager allows them through.

  - sdc_intersection_4carStopSign.world
  - sdc_intersection_8carStopSign.world

Reservations intersection worlds:
These worlds show off intersection management using a reservation policy.
Cars will approach the intersection and make a request to access the intersection
to the intersection manager. The manager will predict the cars path and determine if
any other cars will be in the way. If not it will allow the car through the intersection,
otherwise the car will not be allowed into the intersection and keep making requests until
it can get through.
In order to get data on the performance of our algorithms you should uncomment the print statements in
sdcCar.cc in the init() method (currently lines 1900 through 1923).

  - sdc_intersection_4carReservations.world
  - sdc_intersection_8carReservations.world
  - sdc_intersection_1car.world (only 1 car so this world can be used to get timing data with no traffic)


This world demonstrates the ability of our car to follow lanes and pass other cars
The car navigates two curves, between which is a stretch of straight road that the car navigates while
adjusting its position to stay in the center of the lane. After the second curve, the car reaches
another stretch of straight road, and a second car in the right lane will start driving at a slower speed
than the first car. The first car will then overtake: it will close in on the second car, change to the left
lane, pass the car, and change back.

  - Driving_And_Overtaking_Test.world



This world shows off the ability of our cars to go from intersections, to lane driving, to overtaking.
Combined world:

  - Intersection_And_Driving_Test.world
