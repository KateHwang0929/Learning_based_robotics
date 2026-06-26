# EKF-SLAM for a 2D Kinematic Vehicle

## 1. Overview

This project implements EKF-SLAM for a 2D vehicle. The goal is to estimate both the vehicle state and the landmark positions using noisy bearing and range measurements.

SLAM stands for Simultaneous Localization and Mapping. In this problem, the vehicle needs to estimate its own position while also estimating the positions of five landmarks.

## 2. State Vector

The full state vector contains the vehicle state and landmark positions:

$$
x =
[x, y, \psi, V, l_{1x}, l_{1y}, ..., l_{5x}, l_{5y}]^T
$$

where:

- \(x, y\) are the vehicle position
- \(\psi\) is the heading angle
- \(V\) is the vehicle speed
- \(l_{ix}, l_{iy}\) are the positions of the landmarks

## 3. Motion Model

The vehicle follows the 2D kinematic model:

$$
\dot{x} = V\cos\psi
$$

$$
\dot{y} = V\sin\psi
$$

$$
\dot{\psi} = u_1
$$

$$
\dot{V} = u_2
$$

In this simulation, the yaw rate and acceleration are set to zero, so the vehicle moves forward with constant heading and speed.

## 4. Measurement Model

The vehicle measures:

- its heading angle \(\psi\)
- its speed \(V\)
- the bearing angle \(\beta_i\) to each landmark
- the range \(\rho_i\) to each landmark

For each landmark:

$$
\beta_i = \tan^{-1}\left(\frac{l_{iy} - y}{l_{ix} - x}\right) - \psi
$$

$$
\rho_i = \sqrt{(l_{ix} - x)^2 + (l_{iy} - y)^2}
$$

## 5. EKF-SLAM Process

At each time step, the algorithm performs two steps.

### Prediction Step

The EKF predicts the next vehicle state using the motion model.

### Update Step

The EKF compares the predicted measurement with the actual noisy measurement. Then it corrects the estimated vehicle pose and landmark positions.

## 6. Sensor Accuracy Experiment

The sensor accuracy is controlled by the measurement noise covariance matrix \(R\).

Original setting:

```matlab
bearing_noise = 3*DTR;
range_noise = 0.2;