# Sliding_observation

![Language](https://img.shields.io/badge/language-C%2B%2B%20%7C%20Python-blue.svg)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

A dual-language (**Python** and **C++**) high-performance state estimation framework designed to calculate vehicle/robot sliding parameters in off-road environments. 

This repository implements advanced state estimators to decouple vehicle slip from rough terrain geometry, ensuring accurate odometry and path tracking.

---

## 📌 Features & Observers

The estimation framework is divided into two decoupled estimation layers:

### 1. Nonlinear Observer of Sideslip Angles
* Tracks non-linear lateral slip dynamics and wheel-ground interaction variations.
* Computes real-time dynamic slip angles ($\alpha_f, \alpha_r$) without requiring direct optical ground-speed sensors.
* Robust against sudden changes in surface friction coefficients ($\mu$).

### 2. Luenberger Observer of Ground Geometry
* Estimates real-time terrain topology (longitudinal slope, cross-slope, and local roll/pitch perturbations).
* Uses deterministic linear state-space tracking to filter out high-frequency gravity vector noise caused by chassis vibrations.

---

## 🛠️ Framework Architecture

The framework passes noisy sensor telemetry (IMU, wheel encoders) into the concurrent observer loop to isolate steering slip from physical terrain slopes:
