# Filter Performance Envelope
## Deriving What the Pipeline Can and Cannot Track

This document uses the filter derivations to justify specific claims about the system's observable range. The core argument is: the minimum displacement GPS can meaningfully contribute to the Kalman update, and the minimum speed at which GPS course bearing becomes reliable for yaw correction, are both derivable directly from sensor noise parameters — not empirical guesses.

---

## 1. The GPS Noise Floor Sets a Hard Resolution Limit

The GPS measurement model in the Kalman update is:

```
z = p_true + v_gps       where v_gps ~ N(0, R),  R = 25 m²
```

The GPS reading does not directly report `p_true`. It reports `p_true` plus a noise term drawn from a Gaussian with standard deviation √R = 5 m. This is what we observed experimentally (horizontal scatter of ~12 m between consecutive stationary fixes corresponds to about ±6 m, consistent with √R ≈ 5 m as a 1σ figure).

Now suppose the device has actually moved a distance Δp from its last known position. The Kalman innovation is:

```
y = z - p_estimated = (p_true + v_gps) - p_estimated
```

If the filter's position estimate is reasonably good (low Ppp), this simplifies to:

```
y ≈ Δp + v_gps
```

The innovation is the sum of the true displacement signal and GPS noise. The filter cannot distinguish between these two contributions. The signal-to-noise ratio of the innovation is:

```
SNR = Δp / √R = Δp / 5 m
```

| Δp (true displacement) | SNR | Filter interpretation |
|---|---|---|
| 1 m | 0.2 | Noise-dominated. The 1 m move looks identical to GPS jitter. |
| 5 m | 1.0 | Marginal. Signal and noise are equal magnitude. |
| 10 m | 2.0 | Detectable. Signal is twice the noise floor. |
| 25 m | 5.0 | Clear. GPS update strongly and correctly pulls the estimate. |
| 50 m | 10.0 | Unambiguous. Filter converges rapidly. |

The ~5–10 m minimum detectable displacement is not a design choice — it is a direct consequence of √R = 5 m. No amount of filter tuning moves this threshold, because it is set by the GPS hardware's measurement noise, not by the filter's parameters.

---

## 2. What the Kalman Gain Does at Sub-Noise Displacements

The Kalman gain was derived as:

```
Kp = Ppp / (Ppp + R)
```

Consider what happens when a GPS update arrives after a small true displacement Δp ≈ 2 m, but the position uncertainty Ppp has grown (say to 10 m² after a few seconds of IMU drift). The innovation is:

```
y ≈ Δp + v_gps = 2 + v_gps     where v_gps ~ N(0, 25)
```

The expected innovation is 2 m, but the standard deviation of that innovation is √(Ppp + R) = √35 ≈ 5.9 m. The filter has no statistical basis to attribute `y` to a real 2 m displacement rather than to GPS noise — both are plausible explanations. The gain:

```
Kp = 10 / (10 + 25) = 0.286
```

applies 28.6% of a noisy innovation to the position estimate. If y happens to be +4 m (a plausible GPS noise draw even if the device didn't move), the filter moves its estimate by 0.286 × 4 = 1.14 m. This correction is not meaningful — it is noise being amplified into the state.

By contrast, after a true 25 m displacement where Ppp has grown to 20 m²:

```
y ≈ 25 + v_gps       (expected 25 m, std dev √45 ≈ 6.7 m)
Kp = 20 / (20 + 25) = 0.44
Δstate = 0.44 × 25 ≈ 11 m   (meaningful, correct-direction correction)
```

At this scale the innovation magnitude far exceeds the GPS noise, so the Kalman gain is applying a correction that is genuinely signal.

**Summary:** below ~10 m of true displacement, GPS innovations are statistically indistinguishable from noise, and the filter's updates are unreliable in direction. Above ~10 m, the signal dominates the noise and GPS updates are meaningful. This is the derivation-backed justification for the 5–10 m minimum detectable displacement claim.

---

## 3. Why GPS Course Bearing Requires Speed Above ~2 m/s

GPS course bearing is not measured directly. It is computed inside the GPS module from the change in position between successive fixes (or from Doppler-shifted carrier frequency). Either way, the course is derived from a displacement vector, and that vector is subject to position noise.

**Bearing error from position noise:**

Between two GPS fixes separated by time `T_gps = 0.25 s` (the 4 Hz update rate), a device moving at speed `v` travels a vector with magnitude:

```
Δd = v · T_gps = v · 0.25 m      (in meters, for v in m/s)
```

Each endpoint of that vector has GPS noise with standard deviation σ_p ≈ 5 m. The uncertainty in the displacement vector is therefore on the order of σ_p (the noise on the endpoint, since the start point was already a noisy measurement). The angular error in the bearing estimate is approximately:

```
σ_bearing ≈ arctan(σ_p / Δd) = arctan(5 / (v · 0.25))
```

Evaluate at several speeds:

| Speed v (m/s) | Δd per fix (m) | σ_bearing (approx) |
|---|---|---|
| 0.5 m/s | 0.125 m | arctan(5/0.125) ≈ 88° |
| 1.0 m/s | 0.25 m  | arctan(5/0.25)  ≈ 87° |
| 2.0 m/s | 0.5 m   | arctan(5/0.5)   ≈ 84° |
| 5.0 m/s | 1.25 m  | arctan(5/1.25)  ≈ 76° |
| 10 m/s  | 2.5 m   | arctan(5/2.5)   ≈ 63° |

These are still very large angular errors. The reason the code's 2 m/s threshold works in practice is that real GPS modules perform internal Kalman filtering and Doppler-based velocity estimation before reporting course, substantially improving bearing accuracy compared to the naive position-difference estimate above. The u-blox NEO datasheet specifies course accuracy of approximately 0.5° at speeds above ~5 m/s (Doppler-dominated regime) but degrades sharply below about 1–2 m/s where the Doppler measurement becomes unreliable and the module falls back to position differencing.

**The threshold in the code therefore has two compounding justifications:**

1. Below 2 m/s, GPS course accuracy (even with Doppler) is too poor to trust as a yaw reference. The `ahrs_correct_yaw` gate `gps.speed.mps() > 2.0f` is essentially requiring a minimum displacement signal-to-noise in the velocity domain before accepting a heading.

2. The yaw correction is applied as a soft 5% nudge per GPS fix. Even a slightly wrong heading correction is not catastrophic — the Madgwick gyro integration dominates orientation on short timescales — but below 2 m/s the bearing noise is large enough that repeated "corrections" could actively degrade yaw rather than improve it.

**Yaw drift without a bearing fix:**

The gyroscope has a bias after startup calibration of approximately 0.01–0.05°/s residual (typical for MPU6050). Madgwick's accelerometer correction handles pitch and roll but cannot observe yaw. Without GPS course correction, yaw drifts at that bias rate:

```
yaw drift after 30 s ≈ 0.05°/s × 30 s = 1.5°
yaw drift after 5 min ≈ 0.05°/s × 300 s = 15°
```

A 15° yaw error means the world-frame acceleration projection is wrong by sin(15°) ≈ 26% for horizontal components — which directly corrupts the Kalman position estimate. This establishes why the GPS course correction matters for sustained operation and why the pipeline is explicitly designed for a regime where the device periodically exceeds 2 m/s to collect valid yaw updates.

---

## 4. The IMU Between GPS Fixes: What It Provides and What It Costs

Between GPS updates (every 0.25 s at 4 Hz), the Kalman filter runs predict-only at 100 Hz. The position uncertainty grows during this window according to:

```
ΔPpp ≈ Q_pp × N_steps + (from cross terms) ≈ σ_a² · (T_gps)⁴ / 4
```

With σ_a = 0.05 m/s² and T_gps = 0.25 s:

```
ΔPpp ≈ 0.0025 × (0.25)⁴ / 4 = 0.0025 × 0.000977 = 2.4 × 10⁻⁶ m²
```

The position uncertainty growth in 0.25 s is about 1.6 mm (√2.4×10⁻⁶). This is tiny compared to GPS noise of 5 m — meaning the IMU integrations between fixes are very accurate on short timescales. The IMU's job in the pipeline is smooth inter-fix interpolation, not standalone navigation.

However, if GPS is lost for an extended period, the uncertainty grows quadratically. After T seconds with no GPS:

```
Ppp(T) ≈ Ppp(0) + σ_a² · T⁴ / 4 + (velocity drift terms) ≈ ½ · σ_a · T² ² 
                                                               (the quadratic growth regime)
```

More practically: the 1σ position error from dead-reckoning is:

```
σ_pos(T) ≈ ½ · σ_a · T²   (dominant term for large T)
         = ½ · 0.05 · T²
```

| T without GPS | σ_pos (dead-reckoning only) |
|---|---|
| 1 s | 0.025 m |
| 5 s | 0.625 m |
| 10 s | 2.5 m |
| 30 s | 22.5 m |
| 60 s | 90 m |

After 10 s without GPS, position uncertainty (2.5 m) is still below the GPS noise floor (5 m), meaning a GPS fix arriving at that point would still be informative. After 30 s, dead-reckoning drift has exceeded the GPS noise floor and the filter's position estimate has degraded significantly. This is consistent with the observed result: the system holds ~1 m accuracy at rest for over 30 s because ZUPT keeps velocity near zero, preventing the quadratic drift term from growing — but the theoretical bound without ZUPT is ~22.5 m at 30 s.

---

## 5. The Operating Envelope, Stated as Derivations

**Claim:** *The minimum displacement GPS can meaningfully detect is roughly 5–10 meters.*

**Derivation:** This follows directly from √R = √25 = 5 m. A true displacement Δp is detectable when Δp / √R ≥ 1 to 2, giving Δp ≥ 5–10 m. At smaller displacements, GPS innovations are statistically consistent with noise draws and cannot be distinguished from them.

**Claim:** *Movements at 10–50 m scale and above are where the full pipeline works as intended.*

**Derivation:** At Δp = 10 m, SNR = 2 — signal starts to dominate noise. At Δp = 50 m, SNR = 10 — GPS updates are unambiguous and the Kalman gain applies a correction that is almost entirely signal. In this range, each GPS fix is a reliable anchor, the covariance is kept in check, and the IMU provides smooth 100 Hz interpolation between fixes with sub-centimeter inter-fix drift (as derived in Section 4).

**Claim:** *Speeds above ~2 m/s are needed for GPS course bearing to become reliable for yaw correction.*

**Derivation:** Below 2 m/s, GPS course is derived from displacements that are much smaller than the GPS position noise floor. The bearing SNR is less than arctan(5/0.5) ≈ 84° of angular uncertainty, making it useless and potentially harmful as a yaw reference. Above 2 m/s the GPS module's Doppler-based estimates become reliable (per u-blox spec: < 1° course error at > 5 m/s). The 2 m/s gate is a conservative threshold protecting against yaw corruption.

**What these bounds mean together:**

The pipeline is designed as a sensor fusion system where GPS provides bounded-error long-range position anchoring and the IMU provides fast short-range interpolation. The 10 m resolution floor means the system cannot be used to track fine-grained motion (walking pace indoors, slow precise maneuvering) — those displacements are invisible to the GPS component and the IMU alone drifts too fast without correction. The system's natural use case is outdoor motion at walking-to-driving speeds where GPS fixes arrive reliably, each one carrying enough signal relative to noise to meaningfully update the state.
