# AMCL Particle Filter for Robot Localization in Unity

This Unity project implements an Adaptive Monte Carlo Localization (AMCL) algorithm using a particle filter to estimate the position and orientation of a robot within a predefined environment. The filter adjusts the number of particles based on the effective sample size (ESS) and performs resampling to improve accuracy, with the option to inject randomly placed particles to prevent false localization due to particle grouping.

Key Features:
- Adaptive resampling with adjustable thresholds for particle count.
- Random particle injection to avoid particle grouping in false positions.
- Real-time visualization of particles and robot’s probable position using Unity's GameObjects.
- Supports robot localization in a 3D map with raycasting for sensor-based distance calculations.
- Simple integration with Unity’s physics and environment for robotic simulation.

This AMCL-based localization system is optimized for Unity environments, designed to work seamlessly with robots, and can be customized for different robot configurations or simulation scenarios.
