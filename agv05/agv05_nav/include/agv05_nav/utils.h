/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_NAV_UTILS_H
#define AGV05_NAV_UTILS_H

#include <cassert>
#include <vector>
#include <math.h>


namespace agv05
{

typedef struct
{
  struct
  {
    float junction;
    float offset;
    float offset_prior_turning_left;
    float offset_prior_turning_right;
  } forward, reverse;
  struct
  {
    float decel;
    float speed;
    float speed_min;
    float distance;
    float tolerance;
    float undershoot;
    float timeout;
    bool persist_error;
  } straight;
  struct
  {
    float jerk;
    float decel;
    float debounce;
  } io_trigger;
} StoppingConfig;

typedef struct
{
  float speed;
  struct
  {
    float speed;
    float distance;
    float kp;
    float timeout;
    struct
    {
      float time;
      float distance;
    } hysteresis;
  } align;
} TurningConfig;

template<class T>
T dot(const std::complex<T>& a, const std::complex<T>& b)
{
  return a.real() * b.real() + a.imag() * b.imag();
}

template<class T>
T cross(const std::complex<T>& a, const std::complex<T>& b)
{
  return a.real() * b.imag() - a.imag() * b.real();
}

// https://rosettacode.org/wiki/Fast_Fourier_transform#C.2B.2B
// Cooley-Tukey FFT (in-place, breadth-first, decimation-in-frequency)
template<class T>
void fft(std::vector< std::complex<T> >& x)
{
  // DFT
  unsigned int N = x.size(), k = N, n;
  double thetaT = M_PI / N;
  std::complex<T> phiT = std::complex<T>(cos(thetaT), -sin(thetaT)), t;

  while (k > 1)
  {
    n = k;
    k >>= 1;
    phiT = phiT * phiT;
    t = 1.0L;
    for (unsigned int l = 0; l < k; l++)
    {
      for (unsigned int a = l; a < N; a += n)
      {
        unsigned int b = a + k;
        std::complex<T> y = x[a] - x[b];
        x[a] += x[b];
        x[b] = y * t;
      }
      t *= phiT;
    }
  }

  // Decimate
  unsigned int m = (unsigned int)log2(N);
  for (unsigned int a = 0; a < N; a++)
  {
    unsigned int b = a;
    // Reverse bits
    b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
    b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
    b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
    b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
    b = ((b >> 16) | (b << 16)) >> (32 - m);
    if (b > a)
    {
      t = x[a];
      x[a] = x[b];
      x[b] = t;
    }
  }
}

// inverse fft (in-place)
template<class T>
void ifft(std::vector< std::complex<T> >& x)
{
  // conjugate the complex numbers
  std::for_each(x.begin(), x.end(), [](std::complex<T>& a){a = std::conj(a);});

  // forward fft
  fft(x);

  std::for_each(x.begin(), x.end(), [&x](std::complex<T>& a)
  {
    // conjugate the complex numbers again
    a = std::conj(a);

    // scale the numbers
    a /= x.size();
  });
}

/* Output state change only when the state has become stable. */
template<class T>
class DigitalInputFilter
{
public:
  DigitalInputFilter(bool initial_state, T threshold, T stalled_threshold = std::numeric_limits<T>::max()) :
    threshold_(threshold), stalled_threshold_(stalled_threshold)
  {
    setState(initial_state);
  }

  bool update(bool state, T elapsed_time)
  {
    stalled_ += elapsed_time;

    if (state_ != state)
    {
      accumulated_ += elapsed_time;
      if (accumulated_ > threshold_)
      {
        setState(state);
      }
    }
    else
    {
      accumulated_ = 0;
    }
    return state_;
  }

  bool getState()
  {
    return state_;
  }
  void setState(bool state)
  {
    state_ = state;
    accumulated_ = 0;
    stalled_ = 0;
  }
  bool isStalled()
  {
    return stalled_ > stalled_threshold_;
  }
  void resetStalled()
  {
    stalled_ = 0;
  }

protected:
  bool state_;
  T accumulated_;
  T stalled_;
  const T threshold_;
  const T stalled_threshold_;
};


/* Just a fancy name for a level- or egde- triggered latch. */
template<class T>
class FlipFlop: public DigitalInputFilter<T>
{
public:
  enum
  {
    DISABLED = 0,
    LEVEL_TRIGGER_LO,
    LEVEL_TRIGGER_HI,
    EDGE_TRIGGER_LO_TO_HI,
    EDGE_TRIGGER_HI_TO_LO,
    NUM_TRIGGER_TYPES,
  };
public:
  FlipFlop(uint8_t type, bool initial_state, T threshold, T stalled_threshold = std::numeric_limits<T>::max()) :
    DigitalInputFilter<T>(initial_state, threshold, stalled_threshold),
    type_(type), latch_(false)
  {}

  bool update(bool state, T elapsed_time)
  {
    bool prev_state = DigitalInputFilter<T>::state_;
    state = DigitalInputFilter<T>::update(state, elapsed_time);

    switch (type_)
    {
    case LEVEL_TRIGGER_LO:
      if (!state)
      {
        latch_ = true;
      }
      break;

    case LEVEL_TRIGGER_HI:
      if (state)
      {
        latch_ = true;
      }
      break;

    case EDGE_TRIGGER_LO_TO_HI:
      if (!prev_state && state)
      {
        latch_ = true;
      }
      break;

    case EDGE_TRIGGER_HI_TO_LO:
      if (prev_state && !state)
      {
        latch_ = true;
      }
      break;
    }
    return latch_;
  }

  bool getLatchState()
  {
    return latch_;
  }
  void clearLatch()
  {
    latch_ = false;
  }

protected:
  uint8_t type_;
  bool latch_;
};


/* Bound incoming speed according to velocity, accleration and jerk limits. */
template<class T>
class VelocitySmoother
{
public:
  explicit VelocitySmoother(T speed_lim = std::numeric_limits<T>::max(),
                            T accel_lim = std::numeric_limits<T>::max(),
                            T decel_lim = std::numeric_limits<T>::max(),
                            T accel_jerk_lim = std::numeric_limits<T>::max(),
                            T decel_jerk_lim = std::numeric_limits<T>::max()) :
    speed_(0),
    speed_lim_(speed_lim),
    accel_(0)
  {
    assert(speed_lim >= 0);
    setAccelerationLimits(accel_lim, decel_lim, accel_jerk_lim, decel_jerk_lim);
  }

  // the speed_lim passed into update function is temporary, ie once only.
  void update(float frequency, T speed_lim = std::numeric_limits<T>::max())
  {
    if (frequency <= 0.0f) return;
    float period = 1.0f / frequency;

    assert(speed_lim >= 0);
    speed_lim = std::min(speed_lim, speed_lim_);

    T speed;
    if (speed_lim > speed_)  // acceleration
    {
      T accel = accel_lim_;
      if (accel_jerk_lim_ < std::numeric_limits<T>::max())
      {
        T accel_delta = accel_jerk_lim_ * period;
        T accel_lim = std::sqrt(accel_jerk_lim_ * (speed_lim - speed_) * 2) - accel_delta;
        accel_lim = std::max(accel_lim, accel_delta);
        accel_lim = std::min(accel_lim, accel_lim_);
        accel = std::min(accel_lim, accel_ + accel_delta);
      }
      speed = std::min(speed_lim, speed_ + accel * period);
    }
    else  // deceleration
    {
      T accel = -decel_lim_;
      if (decel_jerk_lim_ < std::numeric_limits<T>::max())
      {
        T decel_delta = decel_jerk_lim_ * period;
        T decel_lim = std::sqrt(decel_jerk_lim_ * (speed_ - speed_lim) * 2) - decel_delta;
        decel_lim = std::max(decel_lim, decel_delta);
        decel_lim = std::min(decel_lim, decel_lim_);
        accel = std::max(-decel_lim, accel_ - decel_delta);
      }
      speed = std::max(speed_lim, speed_ + accel * period);
    }
    speed = std::max<T>(speed, 0);  // speed_lim >= 0
    accel_ = (speed - speed_) * frequency;
    speed_ = speed;
  }

  operator T() const
  {
    return speed_;
  }
  T getSpeed() const
  {
    return speed_;
  }
  T getAccel() const
  {
    return accel_;
  }
  void setSpeed(T speed)
  {
    assert(speed >= 0);
    speed_ = speed;
    accel_ = 0;
  }
  void addSpeedLimit(T speed_lim)
  {
    assert(speed_lim >= 0);
    speed_lim_ = std::min(speed_lim_, speed_lim);
  }
  void clearSpeedLimit()
  {
    speed_lim_ = std::numeric_limits<T>::max();
  }
  void setAccelerationLimits(T accel_lim, T decel_lim,
                             T accel_jerk_lim = std::numeric_limits<T>::max(),
                             T decel_jerk_lim = std::numeric_limits<T>::max())
  {
    accel_lim = std::abs(accel_lim);
    decel_lim = std::abs(decel_lim);
    accel_jerk_lim = std::abs(accel_jerk_lim);
    decel_jerk_lim = std::abs(decel_jerk_lim);
    accel_lim_ = (accel_lim > 0) ? accel_lim : std::numeric_limits<T>::max();
    decel_lim_ = (decel_lim > 0) ? decel_lim : std::numeric_limits<T>::max();
    accel_jerk_lim_ = (accel_jerk_lim > 0) ? accel_jerk_lim : std::numeric_limits<T>::max();
    decel_jerk_lim_ = (decel_jerk_lim > 0) ? decel_jerk_lim : std::numeric_limits<T>::max();
  }

private:
  T speed_;
  T speed_lim_;
  T accel_;
  T accel_lim_;
  T decel_lim_;
  T accel_jerk_lim_;
  T decel_jerk_lim_;
};

template<class T>
class SignedVelocitySmoother: public VelocitySmoother<T>
{
public:
  bool update(float frequency, T speed_lim, T speed_stop)
  {
    if (VelocitySmoother<T>::getSpeed() <= speed_stop)  // stopped
    {
      sign_ = 0;
    }
    if (!speed_lim || (speed_lim * sign_ < 0))  // stopping or opposite sign
    {
      VelocitySmoother<T>::update(frequency, 0);
    }
    else  // equal sign
    {
      sign_ = speed_lim > 0 ? 1 : -1;
      VelocitySmoother<T>::update(frequency, std::abs(speed_lim));
    }
    return !sign_;
  }

  operator T() const
  {
    return VelocitySmoother<T>::getSpeed() * sign_;
  }

private:
  T sign_ = 0;  // current sign
};

/* Reaching kinematic. */
template<class T>
class ReachingKinematic
{
public:
  explicit ReachingKinematic(T speed_min = 0, T decel_lim = 0, T jerk_lim = 0)
  {
    v_ = std::abs(speed_min);
    a_ = std::abs(decel_lim);
    j_ = std::abs(jerk_lim);

    if (j_ > 0)
    {
      T v = (a_ > 0) ? a_ * a_ / j_ / 2 : v_;
      if ((v_ > 0) && (v_ <= v))
      {
        a_ = std::sqrt(j_ * v_ * 2);
      }
      else
      {
        v_ = v;
      }
      x_ = a_ * a_ * a_ / j_ / j_;
    }
    else
    {
      v_ = 0;
      x_ = 0;
    }
  }
  T distance(T speed, T accel = 0)
  {
    T a = (accel > 0) ? accel : 0;
    T dv = (j_ > 0) ? a * a / j_ : 0;  // 2Δv
    T v = (dv > 0) ? dv / 3 + speed : 0;
    T s = (v > 0) ? v * a / j_ : 0;

    v = speed + dv / 2;
    return (v > 0) ? ((a_ > 0) && (v > (v_ * 2))) ?
           (v / 2 + v_) * v / a_ + s :
           (j_ > 0) ? std::sqrt(v * v * v / j_) + s : 0 : 0;
  }
  T speed(T distance)
  {
    return (distance > 0) ? ((a_ > 0) && (distance > x_)) ?
           std::sqrt(distance * a_ * 2 + v_ * v_) - v_ :
           /*std::*/cbrt(j_ * distance * distance) : 0;
  }

private:
  T x_;
  T v_;
  T a_;
  T j_;
};

}  // namespace agv05

#endif  // AGV05_NAV_UTILS_H
