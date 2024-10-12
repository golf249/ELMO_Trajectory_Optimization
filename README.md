# Europa Lander and Mapping Orbiter (ELMO) - Trajectory Optimization

This repository showcases the trajectory optimization code for the **Europa Lander and Mapping Orbiter (ELMO)** mission. The project is part of **Imperial College London's Group Design Project** in the 3rd year.

## Overview

The code implements optimal trajectory generation for the landing phase of the ELMO mission, utilizing control theory and "Lossless Convexification" method to minimize fuel consumption during descent. 

During the **de-orbit, descent, and landing phase**, the trajectory is split into two main phases:

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

## References

1. Acikmese, B., & Ploen, S. R. (2007). Convex Programming Approach to Powered Descent Guidance for Mars Landing. *Journal of Guidance, Control, and Dynamics*, 30(5), 1353-1366. Available from: [https://doi.org/10.2514/1.27553](https://doi.org/10.2514/1.27553).

2. Malyuta, D., Reynolds, T. P., Szmuk, M., Lew, T., Bonalli, R., Pavone, M., et al. (2022). Convex Optimization for Trajectory Generation: A Tutorial on Generating Dynamically Feasible Trajectories Reliably and Efficiently. *IEEE Control Systems*, 42(5), 40-113. Free preprint available at [https://arxiv.org/abs/2106.09125](https://arxiv.org/abs/2106.09125). Available from: [https://doi.org/10.1109/mcs.2022.3187542](https://doi.org/10.1109/mcs.2022.3187542).

3. Blackmore, L., A¸cikme¸se, B., & Scharf, D. P. (2010). Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization. *Journal of Guidance, Control, and Dynamics*, 33(4), 1161-1171. Available from: [https://doi.org/10.2514/1.47202](https://doi.org/10.2514/1.47202).

4. A¸cıkme¸se, B., Carson, J. M., & Blackmore, L. (2013). Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem. *IEEE Transactions on Control Systems Technology*, 21(6), 2104-2113.

Feel free to explore the code and adapt it for your own analyses!
