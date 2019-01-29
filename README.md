# Particle Filter

## Overview
This repository contains coded solution, a Particle Filter, for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Particle Filter function overview

### ParticleFilter::init()
The initialization function initializes the required number of particles to the first gps position with random gaussian noise.

### ParticleFilter::prediction()
The prediction function updates the particle position based on the measurements (velocity and heading).

### ParticleFilter::dataAssociation()
The data association function finds the closest landmark to a specific observation for a given list of observations and landmarks.

### ParticleFilter::updateWeights()
The update weights function performs the following for each particle:
* Transforms the observation x, y coordinates from car coordinates to map coordinates relative to the particle.
* Filters the landmarks within sensor range of the particle
* For each observation, finds the closest landmark
* For each observation calculate the probabilistic weight using the multivariate probability function
* Calculate the weight for each particle by multiplying all the weight for each observation

### ParticleFilter::resample()
The resampling function to "filters out" particles with smaller weights and re-samples particles with higher weights. This is implemented using the discrete distribution function.

## Tests
Once the particle filter program have been developed, I proceeded to test and optimize my implementation to the project criteria by varying the number of particles.

Below are my results: 

| Number of Particles | Elapse time | X error | Y error | Yaw error |
| :---: | :---: | :---: | :---: | :---: |
|   7   | 56.36 | 0.414 | 0.463 | 0.016 |
|   10  | 60.28 | 0.383 | 0.309 | 0.011 |
|   25  | 63.49 | 0.396 | 0.304 | 0.011 |
|   50  | 64.71 | 0.313 | 0.284 | 0.009 |
|  100  | 66.56 | 0.275 | 0.252 | 0.008 |
|  200  | 74.49 | 0.259 | 0.237 | 0.007 |
|  500  | 74.97 | 0.242 | 0.219 | 0.007 |
|  750  | 88.48 | 0.238 | 0.211 | 0.006 |
|  900  | 98.09 | 0.234 | 0.217 | 0.006 |

### Conclusion
Based on my test, I've choose a particle size of 100 as anything less than 100 yields a much higher error and anything higher than 100 yield a diminishing improvement to the errors and increasing the elapsed time.


