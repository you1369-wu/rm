#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "../applications/tools/gravity_compensation.hpp"

namespace
{
void check(bool result, const char * message)
{
  if (!result) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
}

int main()
{
  constexpr float pi = 3.14159265358979323846f;
  constexpr double mass = 0.229, gravity = 9.81;
  constexpr double x0 = (-161.414 - (-173.07)) * 0.001;
  constexpr double z0 = (954.657 - 1052.06) * 0.001;
  const double maximum = mass * gravity * std::hypot(x0, z0);
  check(std::fabs(yuntai::calculate_gravity_compensation(0.0f) - 0.02618508744) < 1e-6,
        "horizontal reference requires positive lifting torque");
  check(std::fabs(yuntai::calculate_gravity_compensation(pi / 2.0f) - 0.21881486547) < 1e-6,
        "positive 90-degree lift uses the original vertical COM offset");

  for (int i = -360; i <= 360; ++i) {
    const double delta = i * static_cast<double>(pi) / 180.0;
    const float torque = yuntai::calculate_gravity_compensation(static_cast<float>(delta));
    const double lever_arm = x0 * std::cos(delta) - z0 * std::sin(delta);
    check(std::fabs(torque - mass * gravity * lever_arm) < 1e-6,
          "implementation agrees with the rotated-coordinate lever arm");
    check(std::fabs(torque) <= maximum + 1e-6, "torque is bounded by m*g*axis_radius");
    check(std::fabs(torque - yuntai::calculate_gravity_compensation(
                              static_cast<float>(delta) + 2.0f * pi)) < 1e-6,
          "one encoder revolution does not change physical compensation");

    const double h = 1e-5;
    const auto energy = [&](double angle) {
      return mass * gravity * (x0 * std::sin(angle) + z0 * std::cos(angle));
    };
    const double energy_derivative = (energy(delta + h) - energy(delta - h)) / (2.0 * h);
    check(std::fabs(torque - energy_derivative) < 1e-6,
          "motor compensation opposes gravity according to potential energy");
  }

  check(!std::isfinite(yuntai::calculate_gravity_compensation(
          std::numeric_limits<float>::quiet_NaN())), "invalid angle remains detectable");
  std::cout << "PASS: gravity magnitude, direction, energy, periodicity and invalid input\n";
}
