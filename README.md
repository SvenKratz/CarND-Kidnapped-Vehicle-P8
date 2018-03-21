# Kindapped Vehicle (Particle Filter) Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


This repository contains my solution to the Udacity Self-Driving Car NanoDegree Unscented Kalman Filter Project.

The repository contains the following notable files:

* [WRITEUP.md](WRITEUP.md) Project writeup report with additional details on my implementation.
* `src/particle_filter.cpp` : Implementation of particle filter for the project
* `src/transform.hpp` : utility header/implementation containing conversion methods for transforming between vehicle and map coordinates as well as distance measure calculation
* [sim-after-running.png](sim-after-running.png) image showing final output of simulator after running the filter

## Compilation and Run Instructions

From the project root directory

1. Create a build directory: `mkdir build`
2. `cd build`
3. Run cmake generator: `cmake ..`
4. Run the executable with `./particle_filter`
