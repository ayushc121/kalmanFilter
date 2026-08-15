# State Estimation Derivation
## Madgwick AHRS + Linear Kalman Filter

This document derives every formula used in the firmware from first principles, in the same order the code executes them. Dummy numbers and dimensional analysis are provided throughout so you can build intuition.

---

## Table of Contents

1. [Mathematical Prerequisites](#1-mathematical-prerequisites)
2. [Madgwick AHRS Filter](#2-madgwick-ahrs-filter)
   - 2.1 Quaternion kinematics (gyro integration)
   - 2.2 The objective function
   - 2.3 The Jacobian and gradient
   - 2.4 The full update rule
   - 2.5 Adaptive beta and GPS yaw correction
3. [Linear Kalman Filter](#3-linear-kalman-filter)
   - 3.1 State vector and process model
   - 3.2 Why the filter is linear (no EKF needed)
   - 3.3 Covariance propagation: deriving the closed-form scalar equations
   - 3.4 Process noise Q: where sigma_a comes from
   - 3.5 GPS measurement update: deriving the gain formulas
   - 3.6 ZUPT
4. [Dimensional Analysis Reference](#4-dimensional-analysis-reference)

---

## 1. Mathematical Prerequisites

### Quaternions

A quaternion is a 4-component number used to represent 3D orientation without the gimbal-lock problems of Euler angles. Written:

```
q = [w, x, y, z]   or equivalently   q = w + xi + yj + zk
```

`w` is the scalar (real) part. `[x, y, z]` is the vector (imaginary) part.

**Unit quaternion:** a quaternion with ||q|| = 1. We restrict to unit quaternions because only those represent pure rotations.

**Conjugate:** q* = [w, -x, -y, -z]. For a unit quaternion, q* = q⁻¹.

**Quaternion multiplication (the Hamilton product):** given q1 = [w1, x1, y1, z1] and q2 = [w2, x2, y2, z2]:

```
(q1 ⊗ q2)_w = w1*w2 - x1*x2 - y1*y2 - z1*z2
(q1 ⊗ q2)_x = w1*x2 + x1*w2 + y1*z2 - z1*y2
(q1 ⊗ q2)_y = w1*y2 - x1*z2 + y1*w2 + z1*x2
(q1 ⊗ q2)_z = w1*z2 + x1*y2 - y1*x2 + z1*w2
```

Note: ⊗ is **not** commutative — order matters.

**Rotating a vector with a quaternion (sandwich product):** to rotate a 3D vector v = [vx, vy, vz] using quaternion q:

```
v_rotated = q ⊗ [0, vx, vy, vz] ⊗ q*
```

Package the vector as a pure quaternion (scalar part = 0), multiply on both sides. The result is another pure quaternion; the vector part `[x, y, z]` of the result is the rotated vector.

**Dummy example:** suppose q = [1, 0, 0, 0] (identity rotation). Then:

```
q ⊗ [0, 3, 4, 5] ⊗ q* = [0, 3, 4, 5]
```

No rotation — the identity quaternion leaves every vector unchanged.

Now suppose q = [cos(π/4), 0, 0, sin(π/4)] = [0.707, 0, 0, 0.707], which is a 90° rotation around the Z-axis:

```
[0, 3, 4, 5] → [0, -4, 3, 5]   (X and Y axes swapped and negated — correct for 90° Z rotation)
```

---

### Covariance Matrices

A covariance matrix P captures how uncertain we are about a state, and how that uncertainty is correlated between different state variables.

For a 2D state [p, v] (position and velocity):

```
P = [var(p)      cov(p,v)]   =   [σ_p²    ρ*σ_p*σ_v]
    [cov(v,p)    var(v)  ]       [ρ*σ_p*σ_v   σ_v²  ]
```

`cov(p,v)` is nonzero when errors in position and velocity are related — which they always are after integrating a noisy accelerometer, because a persistent accelerometer error drives both p and v simultaneously.

P is always symmetric: cov(p,v) = cov(v,p). This means for an N×N symmetric matrix, only N*(N+1)/2 entries are unique. A 3×3 has 6 unique entries — this is why the code stores each axis covariance as a 6-element array.

---

## 2. Madgwick AHRS Filter

**Goal:** maintain a unit quaternion `q` that represents the rotation from body frame to world frame (ENU), updated at 100 Hz.

### 2.1 Quaternion Kinematics (Gyro Integration)

The fundamental relationship between angular velocity and quaternion rate of change is derived from the definition of differentiation on the quaternion manifold. If the body is rotating at angular velocity ω = [ωx, ωy, ωz] rad/s (measured in the body frame), the quaternion evolves as:

```
dq/dt = (1/2) · q ⊗ Ω
```

where `Ω = [0, ωx, ωy, ωz]` is the angular velocity packaged as a pure quaternion.

The factor of 1/2 comes from the double-cover property of quaternions: a 360° physical rotation corresponds to a 720° traversal in quaternion space.

**Expand the product** q ⊗ Ω with q = [w, x, y, z]:

```
(q ⊗ Ω)_w = w*0 - x*ωx - y*ωy - z*ωz = -(x*ωx + y*ωy + z*ωz)
(q ⊗ Ω)_x = w*ωx + x*0  + y*ωz - z*ωy =  w*ωx + y*ωz - z*ωy
(q ⊗ Ω)_y = w*ωy - x*ωz + y*0  + z*ωx =  w*ωy - x*ωz + z*ωx
(q ⊗ Ω)_z = w*ωz + x*ωy - y*ωx + z*0  =  w*ωz + x*ωy - y*ωx
```

**Discretize** using Euler integration with timestep dt:

```
q_{k+1} = q_k + (1/2) · (q_k ⊗ Ω_k) · dt
```

This is exactly what the code computes (`quatdot_gyro[i] *= 0.5f`, then `quaternion[i] += quatdot_gyro[i] * dt`).

**Dummy numbers:** suppose at time k:
```
q = [1, 0, 0, 0]        (facing identity — Z is up, X is east, Y is north)
ω = [0, 0, 10°/s]       → ω in rad/s = [0, 0, 0.1745]
dt = 0.01 s
```

```
q ⊗ Ω = [-(0), 1*0 + 0 - 0, 1*0 - 0 + 0, 1*0.1745 + 0 - 0]
       = [0, 0, 0, 0.1745]

dq/dt = 0.5 * [0, 0, 0, 0.1745] = [0, 0, 0, 0.0873]

q_{k+1} = [1 + 0*0.01, 0 + 0*0.01, 0 + 0*0.01, 0 + 0.0873*0.01]
         = [1, 0, 0, 0.000873]
```

After normalization: `[~1, 0, 0, 0.000873]` — a tiny rotation around Z, consistent with 10°/s for 10 ms.

---

### 2.2 The Objective Function

The gyro integration above will drift because the gyro has a small bias. We need a second source of information to correct orientation.

The accelerometer provides this: when the device is not accelerating linearly, it reads only gravity. In the world frame, gravity points in the +Z direction (ENU convention): `g_world = [0, 0, 1]` (normalized).

If our quaternion estimate is correct, rotating `g_world` into the body frame should match what the accelerometer measures. If they differ, our quaternion estimate is wrong.

**Rotate world-frame gravity into body frame using the current quaternion:**

```
f = q* ⊗ [0, 0, 0, 1] ⊗ q
```

Expand this sandwich product step by step. First compute `q* ⊗ [0, 0, 0, 1]` with q* = [w, -x, -y, -z]:

```
tmp_w = w*0 - (-x)*0 - (-y)*0 - (-z)*1 =  z
tmp_x = w*0 + (-x)*0 + (-y)*1 - (-z)*0 = -y
tmp_y = w*0 - (-x)*1 + (-y)*0 + (-z)*0 =  x
tmp_z = w*1 + (-x)*0 - (-y)*0 + (-z)*0 =  w
```

So `tmp = [z, -y, x, w]`. Now `tmp ⊗ q` with q = [w, x, y, z]:

```
f_w = z*w  - (-y)*x - x*y  - w*z  =  0                   (always 0 — stays pure quaternion)
f_x = z*x  + (-y)*w + x*z  - w*y  =  2xz - 2yw
f_y = z*y  - (-y)*z + x*w  + w*x  =  2yz + 2wx
f_z = z*z  + (-y)*y - x*x  + w*w  =  w² - x² - y² + z²
```

Using the unit quaternion constraint w² + x² + y² + z² = 1, the last component simplifies:

```
f_z = w² - x² - y² + z² = 1 - 2x² - 2y²    (substituting z² = 1 - w² - x² - y², then simplifying)
```

So the predicted accelerometer reading (body-frame gravity) is:

```
f = [0, 2(xz - yw), 2(wx + yz), 1 - 2x² - 2y²]
```

The **objective function** is the error between this prediction and the normalized measured accelerometer reading `[0, ax_hat, ay_hat, az_hat]`:

```
e = [f_x - ax_hat,  f_y - ay_hat,  f_z - az_hat]
  = [2(xz-yw) - ax_hat,  2(wx+yz) - ay_hat,  (1-2x²-2y²) - az_hat]
```

We want to find the direction in quaternion space that, if we move q that way, reduces this error. That direction is the gradient of ||e||² with respect to q.

---

### 2.3 The Jacobian and Gradient

The gradient of ||e||² is `2 · J^T · e` (the 2 gets absorbed into β, so we just compute `J^T · e`).

J is the 3×4 Jacobian matrix of e with respect to q = [w, x, y, z]:

```
     ∂e/∂w   ∂e/∂x   ∂e/∂y   ∂e/∂z
e_x: [ -2y     2z     -2w     2x  ]   (e_x = 2xz - 2yw - ax_hat)
e_y: [  2x     2w      2z     2y  ]   (e_y = 2wx + 2yz - ay_hat)
e_z: [   0    -4x     -4y      0  ]   (e_z = 1-2x²-2y² - az_hat, using constrained form)
```

Note: the constrained form of e_z (which uses 1-2x²-2y² instead of w²-x²-y²+z²) gives ∂e_z/∂w = 0 and ∂e_z/∂z = 0. This is the form Madgwick's paper uses and it removes two terms from the gradient, simplifying computation.

The gradient vector (4×1) is `J^T · e`:

```
∇_w = -2y*e_x + 2x*e_y + 0*e_z
∇_x =  2z*e_x + 2w*e_y - 4x*e_z
∇_y = -2w*e_x + 2z*e_y - 4y*e_z
∇_z =  2x*e_x + 2y*e_y + 0*e_z
```

This is exactly what the code computes (`gradient[0..3]`). Note that only a single matrix-vector multiply is needed — there is no matrix inversion, no iterative solver, no eigendecomposition. The Jacobian was derived analytically beforehand (the code hardcodes these expressions), so at runtime you are just doing 12 multiplications and 8 additions.

---

### 2.4 The Full Update Rule

The Madgwick update subtracts the normalized gradient from the gyro-integrated quaternion rate:

```
dq/dt = (1/2)(q ⊗ Ω) - β · normalize(∇)
```

Integrate:
```
q_{k+1} = q_k + dq/dt · dt
q_{k+1} = normalize(q_{k+1})     (re-enforce unit constraint)
```

The sign is subtraction because we want to descend the gradient — move q in the direction that reduces the error.

**β** (beta) controls the trade-off:
- Large β → accelerometer dominates → orientation tracks gravity well but is noisy during motion.
- Small β → gyro dominates → smooth but drifts slowly.

**Dummy numbers** (at rest, small error):

Suppose:
```
q = [0.999, 0.010, 0.005, 0.000]   (slightly tilted)
ax_hat = [0, 0, 1]                  (accelerometer reads pure gravity — device is actually flat)
```

Compute e:
```
e_x = 2(xz - yw) - 0 = 2(0.010*0.000 - 0.999*0.005) - 0 = -0.00999
e_y = 2(wx + yz) - 0 = 2(0.999*0.010 + 0.005*0.000) - 0 =  0.01998
e_z = 1 - 2*0.010² - 2*0.005² - 1 = -2*0.0001 - 2*0.000025 = -0.00025
```

The gradient pushes q toward zero tilt (back toward identity). After a few hundred update steps, pitch and roll converge to the true attitude. Yaw is unobservable this way because rotating around the gravity vector doesn't change the accelerometer reading.

---

### 2.5 Adaptive Beta and GPS Yaw Correction

**Adaptive β:**

During linear acceleration (e.g., braking), the accelerometer reading is not just gravity — it is gravity plus inertial acceleration. If we blindly trust it, we'll corrupt the orientation. The code gates β by how far the accelerometer magnitude is from 1.0g:

```
a_error = |||a|| - 1.0|           (0 at rest, positive when linearly accelerating)
β_eff   = β · max(0,  1 - a_error/0.25)
```

When `a_error > 0.25g`, β_eff = 0 and the filter runs gyro-only — prioritizing smoothness over correction. For `a_error < 0.25g`, correction scales linearly back up to full β.

**GPS yaw correction:**

Since pitch and roll are corrected by gravity but yaw has no gravity reference, GPS course provides the only yaw anchor when moving. The correction applies a small rotation around the world Z-axis.

Extract current yaw from the quaternion (rotation matrix ZYX convention):
```
yaw = atan2(2(wz + xy), 1 - 2(y² + z²))    [radians]
```

Compute angular error between GPS course and current yaw (handling ±180° wrap). Then build a correction quaternion representing a small rotation around Z:

```
δ = error * 0.05       (5% of error per GPS fix — soft correction)
q_correction = [cos(δ/2), 0, 0, sin(δ/2)]
q_new = q_correction ⊗ q_current
```

This nudges yaw by 5% of the error each GPS update without snapping it.

---

## 3. Linear Kalman Filter

### 3.1 State Vector and Process Model

The full state vector is:

```
x = [px, py, pz, vx, vy, vz, bax, bay, baz]^T      (9×1)
     \_______/   \_______/   \____________/
     position    velocity    accel biases
     (m, ENU)    (m/s, ENU)  (m/s², body frame)
```

The kinematics for each axis (say, East/X) are:

```
p_{k+1} = p_k + v_k·dt + ½·a_k·dt²
v_{k+1} = v_k + a_k·dt
b_{k+1} = b_k                         (bias modeled as constant + random walk)
```

where `a_k` is the world-frame acceleration **after** bias subtraction and gravity removal.

In matrix form, writing the state transition for one axis as [p, v, b]^T:

```
[p]       [1   dt   -½dt²] [p]       [½dt²]
[v]   =   [0    1     -dt] [v]   +   [dt  ] · a_measured
[b]       [0    0       1] [b]       [0   ]

 x_{k+1}  =   F · x_k    +     B    · u_k
```

where `a_measured` is the world-frame acceleration (from rotating the IMU reading through q, then subtracting gravity × 9.81 m/s²). `F` is the state transition matrix and `B` is the control input matrix.

The **−½dt²** and **−dt** entries in column 3 of F capture the fact that if the bias estimate is too large, the corrected acceleration is too small, causing under-prediction of position and velocity. The Kalman filter will use GPS fixes to correct this bias estimate over time.

**Dummy numbers:**

Let dt = 0.01 s, and suppose at rest but with 0.05 m/s² of residual bias in the X axis:

```
x = [0, 0, 0.05]^T    (p=0, v=0, b=0.05 m/s²)
a_measured = 0.05 m/s² (raw — bias hasn't been subtracted yet, is read from state[6])
```

After subtracting bias: a_corrected = 0.05 - 0.05 = 0 m/s². State stays at rest. ✓

If instead bias estimate is zero (b=0) but true bias = 0.05 m/s²:

```
p_{k+1} = 0 + 0*0.01 + ½*0.05*0.0001 = 0.0000025 m per step
```

Over 100 steps (1 second): p ≈ ½ × 0.05 × 1² = 0.025 m of drift per second. After 60 s: ~90 m drift. This is why tracking bias in the state matters so much.

---

### 3.2 Why the Filter Is Linear (No EKF Needed)

An Extended Kalman Filter is required when either the process model or measurement model is nonlinear, because the EKF has to linearize them around the current estimate at every step.

Here:
- **Process model:** pure Newtonian kinematics. F is a constant matrix (independent of state). Linear. ✓
- **Measurement model:** GPS measures position directly. H = [1, 0, 0] per axis. Linear. ✓
- **Orientation:** handled entirely by the Madgwick filter, outside the Kalman state. By the time acceleration reaches the Kalman filter, it has already been rotated into world frame. The Kalman filter never sees the quaternion.

If orientation were a Kalman state, the process model for the quaternion would involve q ⊗ ω, which is nonlinear in q. That would require the EKF. The two-stage design avoids this entirely.

---

### 3.3 Covariance Propagation: Deriving the Closed-Form Scalar Equations

The covariance update rule in the Kalman predict step is:

```
P_{k+1} = F · P_k · F^T + Q
```

This is normally a matrix multiplication requiring O(N³) operations. For our 3×3 per-axis system, that's 27 multiplications just for F·P·F^T. But because F has a specific sparse structure, we can expand this symbolically once and hardcode the result as scalar arithmetic.

**The per-axis state transition matrix F:**

```
F = [1   dt   -h ]     where h = ½dt²
    [0    1   -dt]
    [0    0    1 ]
```

**The symmetric covariance matrix P** (6 unique entries stored as {Ppp, Ppv, Ppb, Pvv, Pvb, Pbb}):

```
P = [Ppp   Ppv   Ppb]
    [Ppv   Pvv   Pvb]
    [Ppb   Pvb   Pbb]
```

**Step 1: Compute M = F · P** (rows of F dotted with columns of P):

```
M[0,:] = F[0,:] · P = [Ppp + dt·Ppv - h·Ppb,   Ppv + dt·Pvv - h·Pvb,   Ppb + dt·Pvb - h·Pbb]
M[1,:] = F[1,:] · P = [Ppv - dt·Ppb,            Pvv - dt·Pvb,           Pvb - dt·Pbb         ]
M[2,:] = F[2,:] · P = [Ppb,                      Pvb,                    Pbb                  ]
```

(Used symmetry: P[1,0] = Ppv, P[2,0] = Ppb, P[2,1] = Pvb.)

**Step 2: Compute M · F^T** (only upper triangle needed due to symmetry of result):

F^T = columns of F:
```
F^T = [1    0   0 ]
      [dt   1   0 ]
      [-h  -dt  1 ]
```

New Ppp = M[0,:] · F^T[:,0] = M[0,0]·1 + M[0,1]·dt + M[0,2]·(-h):
```
= (Ppp + dt·Ppv - h·Ppb) + dt·(Ppv + dt·Pvv - h·Pvb) + (-h)·(Ppb + dt·Pvb - h·Pbb)
= Ppp + 2dt·Ppv - 2h·Ppb + dt²·Pvv - 2dt·h·Pvb + h²·Pbb
```

New Ppv = M[0,:] · F^T[:,1] = M[0,0]·0 + M[0,1]·1 + M[0,2]·(-dt):
```
= (Ppv + dt·Pvv - h·Pvb) + (-dt)·(Ppb + dt·Pvb - h·Pbb)
= Ppv + dt·Pvv - h·Pvb - dt·Ppb - dt²·Pvb + dt·h·Pbb
```

New Ppb = M[0,:] · F^T[:,2] = M[0,0]·0 + M[0,1]·0 + M[0,2]·1:
```
= Ppb + dt·Pvb - h·Pbb
```

New Pvv = M[1,:] · F^T[:,1] = M[1,1]·1 + M[1,2]·(-dt):
```
= (Pvv - dt·Pvb) + (-dt)·(Pvb - dt·Pbb)
= Pvv - 2dt·Pvb + dt²·Pbb
```

New Pvb = M[1,:] · F^T[:,2] = M[1,2]·1:
```
= Pvb - dt·Pbb
```

New Pbb = M[2,:] · F^T[:,2] = M[2,2]·1:
```
= Pbb
```

These six scalar expressions are exactly what `updateCovariance()` computes, plus Q added on the diagonal terms. The matrix multiply has been reduced to 6 scalar formulas with no runtime branching, no memory allocations, and no matrix library.

**Dimensional check:** All P entries have units of (units of state variable)². Since [p] = m, [v] = m/s, [b] = m/s²:

```
[Ppp] = m²
[Ppv] = m · m/s = m²/s
[Ppb] = m · m/s² = m²/s²
[Pvv] = (m/s)² = m²/s²
[Pvb] = (m/s) · (m/s²) = m²/s³
[Pbb] = (m/s²)² = m²/s⁴
```

Check that new Ppv = Ppv + dt·Pvv - h·Pvb - ... has consistent units:
```
[Ppv] = m²/s
[dt·Pvv] = s · m²/s² = m²/s  ✓
[h·Pvb] = s² · m²/s³ = m²/s  ✓
[dt·Ppb] = s · m²/s² = m²/s  ✓
```
All terms have units m²/s. ✓

---

### 3.4 Process Noise Q: Where sigma_a Comes From

The process noise matrix Q represents the uncertainty added to the state during each prediction step due to unmodeled forces (accelerometer noise) and bias drift.

**Continuous-time model:** the acceleration noise is a white noise process `n_a(t)` with power spectral density `σ_a²` (units: (m/s²)²/(rad/s) = m²/s³). The bias drifts as a random walk with spectral density `σ_b²` (units: m²/s⁵).

**Discretizing to get Q:** the noise enters the position and velocity states through the same B matrix used for the control input `[½dt², dt, 0]^T`. The discrete Q is:

```
Q_from_accel = σ_a² · B · B^T  =  σ_a² · [½dt²] · [½dt², dt, 0]
                                           [dt  ]
                                           [0   ]

           = σ_a² · [dt⁴/4    dt³/2   0]
                     [dt³/2    dt²     0]
                     [0        0       0]
```

The bias random walk adds separately to the Pbb entry:

```
Q = [σ_a²·dt⁴/4    σ_a²·dt³/2    0      ]
    [σ_a²·dt³/2    σ_a²·dt²      0      ]
    [0             0             σ_b²·dt]
```

In code notation:
```
Q_pp = σ_a² · dt⁴/4
Q_pv = σ_a² · dt³/2
Q_vv = σ_a² · dt²
Q_bb = σ_b² · dt
```

**Dimensional check** with `σ_a = 0.05 m/s²` and `dt = 0.01 s`:

```
Q_pp = 0.0025 · (0.01)⁴/4 = 0.0025 · 2.5e-9 = 6.25e-12 m²          (very small — position barely grows)
Q_pv = 0.0025 · (0.01)³/2 = 0.0025 · 5e-7   = 1.25e-9  m²/s
Q_vv = 0.0025 · (0.01)²   = 0.0025 · 1e-4   = 2.5e-7   m²/s²
Q_bb = σ_b² · dt = (0.0005)² · 0.01 = 2.5e-9  m²/s⁴ · s = 2.5e-9 m²/s⁴·s
```

The units of Q_bb are [(m/s²)² · s] = m²/s³ — the bias uncertainty grows slowly each step, allowing the filter to track slow drift.

**Intuition for σ_a tuning:**

If σ_a is too large, Q_vv dominates and the filter thinks the IMU is very noisy → it trusts GPS more → position is sluggish but stable.

If σ_a is too small, the filter overtrusts the IMU → position tracks IMU quickly but drifts between GPS fixes.

The value `σ_a = 0.05 m/s²` was tuned empirically: it corresponds to saying "I believe my world-frame acceleration is accurate to within about 5 mg after bias removal."

---

### 3.5 GPS Measurement Update: Deriving the Gain Formulas

When a GPS fix arrives, it provides a noisy measurement of position. The measurement model is:

```
z = H · x + v     where v ~ N(0, R)
```

For one axis: H = [1, 0, 0] (we observe only position, not velocity or bias). R = GPS position variance (e.g., R = 25 m² → 5 m standard deviation).

**The Kalman gain** minimizes the posterior covariance. The derivation:

```
K = P · H^T · (H · P · H^T + R)^{-1}
```

With H = [1, 0, 0] (a row vector):

```
H^T = [1]     (column vector, 3×1)
      [0]
      [0]

P · H^T = [Ppp · 1 + Ppv · 0 + Ppb · 0]   =   [Ppp]
          [Ppv · 1 + Pvv · 0 + Pvb · 0]         [Ppv]
          [Ppb · 1 + Pvb · 0 + Pbb · 0]         [Ppb]

H · P · H^T = [1, 0, 0] · [Ppp, Ppv, Ppb]^T = Ppp    (scalar!)

S = H · P · H^T + R = Ppp + R    (scalar innovation covariance)
```

Since S is a scalar, the matrix inversion `(...)^{-1}` is just `1/S`. So:

```
K = [Ppp, Ppv, Ppb]^T / (Ppp + R)   =   [Kp, Kv, Kb]^T
```

Three scalar gains, computed with two additions and three divisions. No matrix inversion.

**Physical interpretation of the gains:**

- `Kp = Ppp / (Ppp + R)`: how much to correct position. If Ppp >> R (we trust GPS more than IMU), Kp → 1 (full GPS correction). If Ppp << R, Kp → 0 (GPS is too noisy to be useful).
- `Kv = Ppv / (Ppp + R)`: how much to correct velocity using the position measurement. This is nonzero whenever position and velocity errors are correlated — which they always are after drifting under IMU integration. This cross-covariance term is why the filter can infer velocity correction from a position-only GPS measurement.
- `Kb = Ppb / (Ppp + R)`: how much to correct bias. Same logic — the GPS indirectly observes bias through the accumulated position error.

**Dummy numbers:**

Suppose after 10 seconds of dead-reckoning with no GPS:
```
Ppp = 50 m²     (position uncertainty has grown — we've been integrating)
Ppv = 5 m²/s    (position and velocity errors are correlated)
Ppb = 0.2 m²/s² (position error is weakly correlated with bias)
R = 25 m²       (GPS: 5 m std dev)
```

```
S = 50 + 25 = 75 m²
Kp = 50/75 = 0.667    (GPS gets ~2/3 of the weight — plausible after 10 s of drift)
Kv = 5/75  = 0.067    (small velocity correction from this GPS fix)
Kb = 0.2/75 = 0.0027  (very small bias correction — bias doesn't change fast)
```

If the GPS innovation (measured position minus predicted position) is `y = +3 m`:
```
Δp = Kp · y = 0.667 · 3 = +2.0 m   (move estimate 2 m toward GPS)
Δv = Kv · y = 0.067 · 3 = +0.2 m/s (also nudge velocity upward — consistent with having drifted east)
Δb = Kb · y = 0.0027 · 3 = +0.008 m/s² (tiny bias correction)
```

**Covariance update:**

After applying the gain, uncertainty shrinks:

```
P_new = (I - K · H) · P
```

With I - KH:
```
I - K·H = [1-Kp   0   0]   (H = [1,0,0] so K·H only affects column 1)
          [-Kv    1   0]
          [-Kb    0   1]
```

Expanding (I - KH) · P:

```
New Ppp = (1-Kp)·Ppp
New Ppv = (1-Kp)·Ppv
New Ppb = (1-Kp)·Ppb
New Pvv = Pvv - Kv·Ppv      ← cross-covariance: GPS corrects velocity uncertainty too
New Pvb = Pvb - Kv·Ppb
New Pbb = Pbb - Kb·Ppb
```

These are the six lines in `update_axis()`. Again: no matrix library. The H = [1,0,0] structure means entire rows and columns of I-KH are zero, and the update collapses to six scalar expressions.

**Dimensional check on new Pvv:**
```
[Pvv] = m²/s²
[Kv·Ppv] = dimensionless · m²/s = m²/s      ← units don't match!
```

Wait — this looks wrong. Let's check Kv's units:

```
[Kv] = [Ppv] / [S] = (m²/s) / m² = 1/s
[Kv · Ppv] = (1/s) · (m²/s) = m²/s²   ✓
```

Kv is not dimensionless — it carries units of 1/s because it maps position innovation (m) to velocity correction (m/s). The gains carry units:

```
[Kp] = m²  / m²    = dimensionless    (maps m → m)
[Kv] = m²/s / m²   = 1/s             (maps m → m/s)
[Kb] = m²/s² / m²  = 1/s²            (maps m → m/s²)
```

This is an important sanity check: the unit of K is always `[state variable] / [measurement variable]`.

---

### 3.6 ZUPT (Zero Velocity Update)

When the device is stationary, we add an artificial measurement: velocity = 0. The measurement model is now `H = [0, 1, 0]` (observing velocity, not position).

Following the same algebra as the GPS update but with H = [0, 1, 0]:

```
P · H^T = [Ppv, Pvv, Pvb]^T

S = H · P · H^T + R = Pvv + R_zupt

Kp = Ppv / (Pvv + R_zupt)
Kv = Pvv / (Pvv + R_zupt)
Kb = Pvb / (Pvv + R_zupt)
```

Innovation: `y = 0 - v = -v` (we observed that velocity should be 0).

The current implementation only applies Kv (velocity correction), leaving Kp and Kb corrections unused. The full derivation shows that Kp and Kb corrections via cross-covariance are also available and would make ZUPT more effective — a known improvement area.

---

### 3.7 The Three-Axis Decoupling (Why We Can Run Three Independent 3×3 Filters)

The full state is 9-dimensional: [px, py, pz, vx, vy, vz, bax, bay, baz]. If we tracked all 9 states jointly, P would be a 9×9 matrix with 45 unique entries, and F would be 9×9.

However, the dynamics are **decoupled per axis**:
- X position depends only on X velocity and X bias.
- Y position depends only on Y velocity and Y bias.
- Z position depends only on Z velocity and Z bias.
- The GPS measurement model for East does not observe North or Up.

This means the full 9×9 system is block-diagonal: the off-diagonal 3×3 blocks are zero and remain zero forever (no coupling term drives them nonzero). We can therefore factor it into three independent 3-state filters — East, North, Up — each with a 3×3 covariance stored as 6 scalars.

This reduces:
- Covariance storage: 45 entries → 18 entries (3 × 6)
- `F·P·F^T` computation: 27 multiplications × 3 axes, but all hardcoded scalars
- Matrix inversion in Kalman gain: still just scalar division (H=[1,0,0])

The result is a filter that runs efficiently on a microcontroller in scalar C++ with no heap allocation and no loops over matrix indices.

---

## 4. Dimensional Analysis Reference

| Symbol | Value | Units | Physical meaning |
|--------|-------|-------|-----------------|
| dt | 0.01 | s | IMU timestep (100 Hz) |
| h = ½dt² | 5×10⁻⁵ | s² | Position-from-accel coefficient |
| σ_a | 0.05 | m/s² | Acceleration noise standard deviation |
| σ_b | 0.0005 | m/s²/√s | Bias random walk intensity |
| Q_pp | σ_a²·dt⁴/4 | m² | Position process noise per step |
| Q_vv | σ_a²·dt² | m²/s² | Velocity process noise per step |
| Q_bb | σ_b²·dt | m²/s⁴·s = m²/s³ | Bias process noise per step |
| R_horiz | 25 | m² | GPS horizontal measurement variance |
| R_vert | 100 | m² | GPS vertical measurement variance |
| Ppp (initial) | 9 | m² | Initial position uncertainty (±3 m) |
| Pvv (initial) | 0.25 | m²/s² | Initial velocity uncertainty (±0.5 m/s) |
| Pbb (initial) | 0.25–0.50 | m²/s⁴·s | Initial bias uncertainty |
| Kp | Ppp/(Ppp+R) | dimensionless | Position Kalman gain (unitless) |
| Kv | Ppv/(Ppp+R) | 1/s | Velocity Kalman gain |
| Kb | Ppb/(Ppp+R) | 1/s² | Bias Kalman gain |
| β (base) | 0.75 | dimensionless | Madgwick correction weight |
| GPS noise (observed) | ~12 | m | Horizontal scatter between fixes at rest |
| Accel drift (observed) | ~35 | m | Dead-reckoning drift within seconds at rest |

**Quick sanity checks to run on any new tuning values:**

1. Q_vv should be much smaller than Pvv_initial (noise added per step is much smaller than starting uncertainty). With the values above: 2.5×10⁻⁷ << 0.25. ✓
2. R should be comparable to Ppp after several seconds without GPS — if Ppp has grown to ~50 m² from dead-reckoning drift, R=25 m² gives roughly equal weight to IMU and GPS. ✓
3. β should be small enough that in one second (100 steps), the gyro hasn't been completely overridden by the accelerometer: effective correction per second is approximately 100·β·dt = 100·0.75·0.01 = 0.75 "units of gradient" — fairly aggressive.
