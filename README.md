# AMCL Simulation Project

Welcome to the **Adaptive Monte Carlo Localization (AMCL)** simulation project! This repository showcases the AMCL algorithm in a robotics localization context, bridging the gap between theoretical probability/statistics concepts and their practical applications in robotics.

## Overview

**AMCL** is a probabilistic algorithm widely used in robotics to localize a robot within a known map. By leveraging Bayesian inference, Monte Carlo methods, and dynamic adjustment of particle counts, AMCL efficiently estimates the robot’s pose—even in the face of uncertainty and noise.

This project provides:

- **A main landing page (Home)** that introduces the project’s purpose and guiding principles.
- **An in-depth Explanation page** detailing the underlying mathematics and theories behind AMCL.
- **An Interactive Simulation page** where users can observe how AMCL behaves in real-time as they control a robot’s movements.

All pages adopt a cohesive dark-themed styling for a modern and readable experience.

## Pages

### 1. Home Page
- **File:** `docs/index.html`
- **What’s Inside:**  
  Introduces the concept of AMCL, providing a high-level understanding of the problem space and linking to the other sections.  
  Features a hero section with a call-to-action button to try the simulation, and a brief project overview encouraging users to learn more via the explanation page.

### 2. Explanation Page
- **File:** `docs/explanation.html`
- **What’s Inside:**  
  Offers an extensive theoretical background on AMCL:
  - Probability and random variables  
  - Bayesian inference (Bayes’ theorem)  
  - Monte Carlo methods and particle filters  
  - Detailed breakdown of states, controls, observations, and the Bayesian filtering process  
  - Explanation of AMCL’s adaptive mechanisms, including ESS and KLD-sampling  
  - References and appendices for further reading

### 3. Simulation Page
- **File:** `docs/simulation.html`
- **What’s Inside:**  
  Embeds a Unity-based WebGL simulation demonstrating AMCL in action.  
  Users can:
  - Control the robot using W, A, S, D or arrow keys.  
  - Observe how particles converge around the robot’s true position over time.  
  - Experience how AMCL handles uncertainty and adapts dynamically.

**Note:** Parameter adjustments are currently unavailable. The goal is to simply experience how the algorithm behaves as the robot moves.

## Getting Started

1. **Clone this repository:**
   ```bash
   git clone https://github.com/kaidalisohaib/AMCL-Visualization.git
   cd amcl-simulation
   ```
