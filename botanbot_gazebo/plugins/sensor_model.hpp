//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
#define HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H

#include "sdf/sdf.hh"
#include "ignition/math/Vector3.hh"

#include <numeric>

namespace gazebo
{

template<typename T>
class SensorModel_
{
public:
  SensorModel_();
  virtual ~SensorModel_();

  virtual void Load(sdf::ElementPtr _sdf, const std::string & prefix = std::string());

  virtual T operator()(const T & value) const {return value * scale_error + current_error_;}
  virtual T operator()(const T & value, double dt) {return value * scale_error + update(dt);}

  virtual T update(double dt);
  virtual void reset();
  virtual void reset(const T & value);

  virtual const T & getCurrentError() const {return current_error_;}
  virtual T getCurrentBias() const {return current_drift_ + offset;}
  virtual const T & getCurrentDrift() const {return current_drift_;}
  virtual const T & getScaleError() const {return scale_error;}

  virtual void setCurrentDrift(const T & new_drift) {current_drift_ = new_drift;}

  virtual void dynamicReconfigureCallback(SensorModelConfig & config, uint32_t level);

private:
  virtual bool LoadImpl(sdf::ElementPtr _element, T & _value);

public:
  T offset;
  T drift;
  T drift_frequency;
  T gaussian_noise;
  T scale_error;

  struct SensorModelConfig
  {
    double offset;
    double drift;
    double drift_frequency;
    double gaussian_noise;
    double scale_error;
  };

private:
  T current_drift_;
  T current_error_;
};

namespace
{
template<typename T>
static inline T SensorModelGaussianKernel(T mu, T sigma)
{
  // using Box-Muller transform to generate two independent standard normally distributed normal variables
  // see wikipedia
  T U = (T)rand() / (T)RAND_MAX; // normalized uniform random variable
  T V = (T)rand() / (T)RAND_MAX; // normalized uniform random variable
  T X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  X = sigma * X + mu;
  return X;
}

template<typename T>
static inline T SensorModelInternalUpdate(
  T & current_drift, T drift, T drift_frequency, T offset,
  T gaussian_noise, double dt)
{
  // current_drift = current_drift - dt * (current_drift * drift_frequency + SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift));
  current_drift = exp(-dt * drift_frequency) * current_drift + dt * SensorModelGaussianKernel(
    T(), sqrt(2 * drift_frequency) * drift);
  return offset + current_drift + SensorModelGaussianKernel(T(), gaussian_noise);
}
}

namespace helpers
{
template<typename T>
struct scalar_value { static double toDouble(const T & orig) {return orig;} };
template<typename T>
struct scalar_value<std::vector<T>> { static double toDouble(const std::vector<T> & orig)
  {
    return (double) std::accumulate(orig.begin(), orig.end()) / orig.size();
  }
};
template<>
struct scalar_value<ignition::math::Vector3d> { static double toDouble(
    const ignition::math::Vector3d & orig)
  {
    return (orig.X() + orig.Y() + orig.Z()) / 3;
  }
};
}

typedef SensorModel_<double> SensorModel;
typedef SensorModel_<ignition::math::Vector3d> SensorModel3;

}

#endif // HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
