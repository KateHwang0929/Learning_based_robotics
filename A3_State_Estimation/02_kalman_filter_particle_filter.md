# 02 Kalman Filter and Particle Filter

## 1. Overview

Kalman Filter and Particle Filter are both used for **state estimation** in robotics.

They both try to answer the same question:

> The robot moved and received noisy sensor data. Where is it probably now?

Both methods follow the same general structure:

1. **Prediction:** use the motion model to predict the next state.
2. **Update:** use the sensor measurement to correct the prediction.

The main difference is how they represent uncertainty.

* **Kalman Filter:** represents belief using a mean and covariance.
* **Particle Filter:** represents belief using many weighted samples called particles.



## 2. Kalman Filter

A Kalman Filter estimates the robot state using:

$$
\text{mean} + \text{covariance}
$$

The **mean** represents the best estimate of the robot state.
The **covariance** represents how uncertain the robot is about that estimate.

For example, a simple robot state can be written as:

$$
x_k =
\begin{bmatrix}
x \
y \
\theta
\end{bmatrix}
$$

where:

* $x$ is the horizontal position
* $y$ is the vertical position
* $\theta$ is the heading angle

The Kalman Filter says:

> I think the robot is here, and this is how uncertain I am.



## 3. Kalman Filter Model

The standard Kalman Filter assumes a **linear system** with Gaussian noise:

$$
x_k = A x_{k-1} + B u_k + w_k
$$

$$
z_k = Hx_k + v_k
$$

where:

* $x_k$ is the current state
* $x_{k-1}$ is the previous state
* $u_k$ is the control input
* $z_k$ is the sensor measurement
* $A$ is the state transition matrix
* $B$ maps the control input to the state
* $H$ maps the state to the measurement
* $w_k$ is process noise
* $v_k$ is measurement noise

The noise terms are usually modeled as Gaussian:

$$
w_k \sim \mathcal{N}(0,Q)
$$

$$
v_k \sim \mathcal{N}(0,R)
$$

where:

* $Q$ is the process noise covariance
* $R$ is the measurement noise covariance



## 4. Kalman Filter Prediction Step

The prediction step uses the motion model to estimate where the robot should be next.

### State Prediction

$$
\hat{x}*{k|k-1} = A\hat{x}*{k-1|k-1} + Bu_k
$$

This means:

> Based on where I was before and how I moved, I predict that I am here now.

### Uncertainty Prediction

$$
P_{k|k-1} = AP_{k-1|k-1}A^T + Q
$$

where:

* $P$ is the covariance matrix
* $Q$ is process noise

The uncertainty usually increases during prediction because real robot motion is imperfect.



## 5. Kalman Filter Update Step

After prediction, the robot receives a sensor measurement.

The filter compares:

$$
\text{actual measurement} - \text{predicted measurement}
$$

This difference is called the **innovation** or **measurement residual**.

### Kalman Gain

$$
K_k = P_{k|k-1}H^T(HP_{k|k-1}H^T + R)^{-1}
$$

The Kalman gain decides how much the estimator should trust the measurement compared to the prediction.

If the sensor is accurate, the Kalman gain gives more weight to the measurement.
If the sensor is noisy, the Kalman gain gives more weight to the prediction.

### State Update

$$
\hat{x}_{k|k}
=============

\hat{x}*{k|k-1} + K_k(z_k - H\hat{x}*{k|k-1})
$$

This can be understood as:

$$
\text{new estimate}
===================

\text{prediction}
+
\text{correction}
$$

### Uncertainty Update

$$
P_{k|k} = (I - K_kH)P_{k|k-1}
$$

The update step usually reduces uncertainty because the robot has received new sensor information.



## 6. Kalman Filter Intuition

The Kalman Filter is like a smart average between:

$$
\text{motion prediction}
+
\text{sensor measurement}
$$

It does not blindly trust either one.

If the sensor is reliable, it trusts the sensor more.
If the sensor is noisy, it trusts the motion model more.

In simple terms, the Kalman Filter asks:

> Should I believe my prediction more, or my measurement more?



## 7. Extended Kalman Filter

The standard Kalman Filter works for linear systems, but many robotics problems are nonlinear.

For example, a robot may move according to:

$$
x_{k+1} = x_k + \cos(\theta_k)
$$

$$
y_{k+1} = y_k + \sin(\theta_k)
$$

$$
\theta_{k+1} = \theta_k
$$

This is nonlinear because it includes:

$$
\cos(\theta_k)
$$

and

$$
\sin(\theta_k)
$$

The **Extended Kalman Filter**, or **EKF**, is a version of the Kalman Filter for nonlinear systems.

Instead of using linear models:

$$
x_k = A x_{k-1} + B u_k + w_k
$$

$$
z_k = Hx_k + v_k
$$

the EKF uses nonlinear models:

$$
x_k = f(x_{k-1}, u_k, w_k)
$$

$$
z_k = h(x_k, v_k)
$$

Then it linearizes the nonlinear functions around the current estimate using Jacobians.

In simple terms:

> EKF is a Kalman Filter adapted for nonlinear robotics problems.



## 8. Particle Filter

A Particle Filter estimates the robot state using many possible guesses.

Each guess is called a **particle**.

Instead of saying:

> The robot is probably at one mean position.

the Particle Filter says:

> Here are many possible robot states. Some are more likely than others.

The belief is represented as:

$$
bel(x_k) \approx {x_k^{[i]}, w_k^{[i]}}_{i=1}^{N}
$$

where:

* $x_k^{[i]}$ is the $i$-th particle
* $w_k^{[i]}$ is the weight of the $i$-th particle
* $N$ is the number of particles

Each particle is one possible state of the robot.

For example:

```text
Particle 1: robot might be at (1.0, 2.0)
Particle 2: robot might be at (1.2, 2.1)
Particle 3: robot might be at (0.8, 1.9)
```



## 9. Particle Filter Steps

A Particle Filter usually has four main steps.

### Step 1: Initialize Particles

Start with many possible guesses of the robot state.

For example, the robot may begin with 100, 500, or 1000 particles.

More particles usually give a better estimate, but they also require more computation.

### Step 2: Prediction

Move every particle using the motion model.

For example:

$$
x_{k+1} = x_k + \cos(\theta_k)
$$

$$
y_{k+1} = y_k + \sin(\theta_k)
$$

$$
\theta_{k+1} = \theta_k
$$

This means:

> Each particle moves one step forward in the direction it is facing.

### Step 3: Weight Update

After receiving a sensor measurement, each particle receives a weight.

If a particle matches the measurement well, it receives a high weight.
If a particle does not match the measurement, it receives a low weight.

Mathematically:

$$
w_k^{[i]} \propto p(z_k \mid x_k^{[i]})
$$

This means:

> A particle gets a higher weight if the sensor measurement is likely from that particle’s state.

### Step 4: Resampling

After weighting, the filter resamples the particles.

Particles with high weights are more likely to survive.
Particles with low weights are more likely to disappear.

In simple terms:

> Bad guesses disappear, and good guesses survive.



## 10. Particle Filter Intuition

A Particle Filter is like throwing many guesses onto a map.

At first, the guesses may be spread out.
After the robot receives sensor measurements, the bad guesses are removed.
Over time, the particles gather around the most likely robot state.

Simple summary:

$$
\text{many guesses}
+
\text{sensor checking}
+
\text{resampling}
=================

\text{better estimate}
$$



In simple terms:

* **Kalman Filter:** one best guess plus uncertainty
* **Particle Filter:** many guesses plus weights

Both methods help a robot estimate where it is, how it is moving, and how confident it should be in that estimate.
