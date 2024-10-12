# Europa Lander and Mapping Orbiter (ELMO) - Trajectory Optimization

This repository showcases the trajectory optimization code for the **Europa Lander and Mapping Orbiter (ELMO)** mission. The project is part of **Imperial College London's Group Design Project** in the 3rd year.

## Overview

The code implements optimal trajectory generation for the landing phase of the ELMO mission, utilizing control theory and 'Lossless Convexification' method to minimize fuel consumption during descent. The main objectives include:

- Powered descent trajectory optimization.
- Simulation of landing dynamics on Europa's surface.

## Code Structure

The codebase includes the following main files:

- `main.m`: The main script for running the trajectory optimization and analysis.
- `pre_descent_stage.m`: Function to compute optimal trajectory for the powered descent stage.
- `solve_pdg_fft_descent.m`: Function to solve the optimal control problem for the powered descent.
- `pre_lander.m`: Function to compute optimal trajectory for the landing stage.
- `solve_pdg_fft_lander.m`: Function to solve the optimal control problem for the landing.
- Additional utility functions for plotting and analysis.

## Prerequisites

To run the code, you will need:

- MATLAB with support for the CVX optimization package. Please visit the [CVX website](http://cvxr.com/cvx/) for installation instructions.

## Running the Code

1. Ensure all required files are in your MATLAB path.
2. Open `main.m` and run the script to execute the trajectory optimization.

## Project Background

This project is part of the ELMO mission, aiming to explore Europa's surface and gather crucial data for future missions. The trajectory optimization code is a key component in ensuring a successful landing.

Feel free to explore the code and adapt it for your own analyses!
