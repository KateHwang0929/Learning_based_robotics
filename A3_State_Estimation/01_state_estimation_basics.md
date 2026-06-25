# 01 State Estimation Basics

## 1. What is State Estimation?

State estimation is the process of estimating the hidden true state of a system by combining a motion model, sensor measurements, and uncertainty.

In robotics, a robot is constantly trying to answer questions such as:

* Where am I?
* Which direction am I facing?
* How fast am I moving?
* How confident am I in this estimate?

A robot usually does not have direct access to its true state. Its sensors are noisy, its camera may be blurry, its wheels may slip, and its GPS may be inaccurate. Therefore, instead of assuming:

> I am exactly here.

the robot maintains a more realistic belief:

> I am probably around here, with this level of confidence.

Robotic systems estimate their state using two main sources of information:

1. **Motion model:** predicts how the robot should move based on its previous state and control input.
2. **Sensor measurements:** provide noisy evidence about the robot’s current state.

State estimation combines these two sources to produce the best estimate of the robot’s current state and its uncertainty.

## 2. State-Space View

A robot can be modeled as a dynamical system:

$$
x_{k+1} = f(x_k, u_k, w_k)
$$

$$
z_k = h(x_k, v_k)
$$

where:

* $x_k$ is the hidden state at time step $k$
* $u_k$ is the control input
* $w_k$ is process noise from imperfect motion
* $z_k$ is the sensor measurement
* $v_k$ is measurement noise from imperfect sensors
* $f$ is the motion model
* $h$ is the measurement model

The goal of state estimation is to infer the hidden state $x_k$ using all previous controls and measurements.

## 3. Belief Representation

In robotics, the robot usually does not know its exact state. Instead, it maintains a belief over possible states:

$$
bel(x_k) = p(x_k \mid z_{1:k}, u_{1:k})
$$

This represents the probability distribution of the current state given all past measurements and control inputs.

A good state estimator should answer two questions:

1. What is the most likely current state?
2. How uncertain is the estimate?

This is important because a robot should not only know where it thinks it is, but also how confident it is in that estimate.

## 4. Recursive Estimation: Prediction and Update

Most state estimation algorithms follow a recursive two-step structure: prediction and update.

### Prediction Step

The estimator first uses the motion model to predict the next state:

$$
\hat{x}*{k|k-1} = f(\hat{x}*{k-1}, u_k)
$$

This step estimates where the robot should be after applying the control input. However, because real motion is imperfect, the prediction usually increases uncertainty.

### Update Step

After receiving a sensor measurement, the estimator corrects the prediction:

$$
\hat{x}*{k|k} = \hat{x}*{k|k-1} + \text{correction}
$$

This step uses measurement information to reduce uncertainty and improve the estimate.

In simple terms:

$$
\text{prediction from motion} + \text{correction from sensors} = \text{better state estimate}
$$

## 5. Why State Estimation Matters

State estimation is essential in robotics because almost every robotic task depends on knowing the state of the robot and environment.

For example:

* Navigation requires estimating the robot’s position.
* Control requires estimating velocity, orientation, and tracking error.
* Mapping requires estimating the positions of landmarks or obstacles.
* SLAM requires estimating both the robot pose and the environment at the same time.

Without reliable state estimation, even a good planner or controller may fail because it is making decisions based on inaccurate information.
