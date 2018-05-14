# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Obeservation

* Three lane and each lane is 4 m. The frenet d starts 0 from left to right. Frenet s starts 0 to max_s
* The simulator process each data every 0.02 s.
* Every second 50 data points will be processed.
* Each cycle the simulator comsumes around 3 data points, i.e., 0.06 s
* There might be some delay caused by our process. During this delay, the simulator will use previous data as guide.
  It also means when we process, the car has already moved to further position.
* Which coordinate system we should use in path planning and trajectory generation, considering for simulator it uses
  global (x, y) coordinate ?
  * It is good to use frenet coordinate for path planning, since we need to know where we are going **on the road**.
  * However, speed, acceleration and jerk are calculated using global coordinate.
    The discrepency between two coordinates might cause inconsistency in speed, acceleration and jerk etc.
  * Car coordinate is used for `spline`, but seems not required for JMT