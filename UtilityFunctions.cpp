#include "UtilityFunctions.hpp"

int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec) {
  // Replace these two coefficients with what you get
  // in the experiment. Note the trailing "f" after the
  // number -- this ensures that we use single precision
  // floating point (rather than double precision, which
  // would be substantially slower on the microcontroller).

  float a = -69.3328f;  // the zeroth order term from our poly fit
  float b = 0.1446f;  // the first order term from our poly fit

  return int(a + b * desiredSpeed_rad_per_sec);
}

float speedFromForce(float desiredForce_N) {
  // replace this with your determined constant:
  // Remember to add the trailing "f" for single
  // precision!
  float const propConstant = 3.8493e-8f; //Implementing our quadratic fit coefficient

  //we implement a safety check,
  //  (no sqrtf for negative numbers)
  if (desiredForce_N <= 0) {
    return 0.0f;
  }

  return sqrtf(desiredForce_N / propConstant); //The function outputs the required speed
}

