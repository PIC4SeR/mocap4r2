// Copyright 2024 PIC4SeR
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOCAP4R2_PEOPLE__CV_KALMAN_HPP_
#define MOCAP4R2_PEOPLE__CV_KALMAN_HPP_

namespace mocap4r2_filters
{

// Scalar constant-velocity Kalman filter.
//
// State x = [position, velocity]^T, measuring position only. Estimating
// velocity from noisy mocap position measurements with a constant-velocity
// motion model (continuous white-noise acceleration). Compared with a plain
// finite-difference + EMA, this gives smoother, lower-lag velocity, is
// naturally dt-aware, and gracefully rides through dropped/late samples.
class CVKalman1D
{
public:
  // q: process-noise spectral density (acceleration variance, (m/s^2)^2 / Hz).
  //    Larger -> trusts the measurements more (more responsive, noisier).
  // r: measurement variance (m^2). Larger -> trusts the model more (smoother).
  explicit CVKalman1D(double q = 1.0, double r = 1e-4)
  : q_(q), r_(r)
  {
    reset(0.0);
  }

  void set_noise(double q, double r)
  {
    q_ = q;
    r_ = r;
  }

  // Re-seed the filter at position p0 with zero velocity and a large velocity
  // covariance (so the first updates move it quickly).
  void reset(double p0)
  {
    x_p_ = p0;
    x_v_ = 0.0;
    P00_ = r_;
    P01_ = 0.0;
    P10_ = 0.0;
    P11_ = 1.0;
    initialized_ = true;
  }

  // Fuse a new position measurement taken dt seconds after the previous one.
  // Returns the updated velocity estimate. dt must be > 0.
  double update(double z, double dt)
  {
    // Predict: x = F x, with F = [[1, dt], [0, 1]].
    x_p_ += dt * x_v_;

    // P = F P F^T + Q
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double P00 = P00_ + dt * (P01_ + P10_) + dt2 * P11_;
    const double P01 = P01_ + dt * P11_;
    const double P10 = P10_ + dt * P11_;
    const double P11 = P11_;
    // Continuous white-noise acceleration process noise.
    P00_ = P00 + q_ * dt3 / 3.0;
    P01_ = P01 + q_ * dt2 / 2.0;
    P10_ = P10 + q_ * dt2 / 2.0;
    P11_ = P11 + q_ * dt;

    // Update with measurement z (H = [1, 0]).
    const double S = P00_ + r_;
    const double K0 = P00_ / S;
    const double K1 = P10_ / S;
    const double y = z - x_p_;
    x_p_ += K0 * y;
    x_v_ += K1 * y;

    // P = (I - K H) P
    const double nP00 = (1.0 - K0) * P00_;
    const double nP01 = (1.0 - K0) * P01_;
    const double nP10 = P10_ - K1 * P00_;
    const double nP11 = P11_ - K1 * P01_;
    P00_ = nP00;
    P01_ = nP01;
    P10_ = nP10;
    P11_ = nP11;

    return x_v_;
  }

  double position() const {return x_p_;}
  double velocity() const {return x_v_;}
  bool initialized() const {return initialized_;}

private:
  double q_;
  double r_;
  double x_p_{0.0};
  double x_v_{0.0};
  double P00_{0.0};
  double P01_{0.0};
  double P10_{0.0};
  double P11_{0.0};
  bool initialized_{false};
};

}  // namespace mocap4r2_filters

#endif  // MOCAP4R2_PEOPLE__CV_KALMAN_HPP_
