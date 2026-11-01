#!/usr/bin/env python3
"""Compile the production mecanum transforms on the host without board libraries."""
from pathlib import Path
import os
import re
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    header = (root / "src/VehicleDrive/VD_vehicle_controller.hpp").read_text()
    source = (root / "src/VehicleDrive/VD_vehicle_controller.cpp").read_text()
    direction = re.search(r"struct Direction \{.*?\n\};", header, re.S).group()
    places = re.search(r"  enum M_Place \{.*?\n  \};", header, re.S).group()
    constants = "\n".join(re.findall(r"  const float [^;]+;", header))
    transforms = re.findall(r"void VEHICLE_CTRL::conv_.*?\n\}", source, re.S)
    assert len(transforms) == 2, "Both production transforms must be tested"
    program = """
#include <cassert>
#include <cmath>
#include <cstdio>
""" + direction + "\nstruct VEHICLE_CTRL {\n" + places + constants + """
  void conv_Vdir_to_Mdir(Direction&, float*);
  void conv_Mdir_to_Vdir(float*, Direction&);
};
""" + "\n".join(transforms) + r'''
void near(float actual, float expected) {
  assert(std::isfinite(actual));
  assert(std::fabs(actual - expected) <= 1e-4f * (1.0f + std::fabs(expected)));
}

int main() {
  VEHICLE_CTRL control;
  auto check_wheels = [&](Direction velocity, float fl, float bl, float br, float fr) {
    float wheels[4] = {};
    control.conv_Vdir_to_Mdir(velocity, wheels);
    near(wheels[VEHICLE_CTRL::FL], fl);
    near(wheels[VEHICLE_CTRL::BL], bl);
    near(wheels[VEHICLE_CTRL::BR], br);
    near(wheels[VEHICLE_CTRL::FR], fr);
  };
  // Independent geometric reference: 170x200 mm wheel spacing, 37.5 mm radius.
  const float yaw_wheel_speed = 185.0f / 37.5f;
  check_wheels({0, 0, 0}, 0, 0, 0, 0);
  check_wheels({0, 0, 1}, -yaw_wheel_speed, -yaw_wheel_speed,
               yaw_wheel_speed, yaw_wheel_speed);
  check_wheels({0, 0, -1}, yaw_wheel_speed, yaw_wheel_speed,
               -yaw_wheel_speed, -yaw_wheel_speed);
  check_wheels({37.5f, 0, 0}, 1, 1, 1, 1);
  check_wheels({0, 37.5f, 0}, -1, 1, -1, 1);
  check_wheels({37.5f, 37.5f, 0}, 0, 2, 0, 2);
  check_wheels({37.5f, -37.5f, 0}, 2, 0, 2, 0);
  check_wheels({37.5f, 0, 1}, 1 - yaw_wheel_speed, 1 - yaw_wheel_speed,
               1 + yaw_wheel_speed, 1 + yaw_wheel_speed);

  float yaw_wheels[] = {-yaw_wheel_speed, -yaw_wheel_speed,
                        yaw_wheel_speed, yaw_wheel_speed};
  Direction yaw = {};
  control.conv_Mdir_to_Vdir(yaw_wheels, yaw);
  near(yaw.x, 0);
  near(yaw.y, 0);
  near(yaw.th, 1);

  // Include both signs, translation, rotation and combined commands.
  const float linear[] = {-400, -100, 0, 100, 400};
  const float angular[] = {-18, -1, 0, 1, 18};
  for (float x : linear) {
    for (float y : linear) {
      for (float th : angular) {
        Direction input = {x, y, th}, output = {};
        float wheels[4] = {};
        control.conv_Vdir_to_Mdir(input, wheels);
        control.conv_Mdir_to_Vdir(wheels, output);
        near(output.x, input.x);
        near(output.y, input.y);
        near(output.th, input.th);
      }
    }
  }
  std::puts("PASS: physical wheel speeds, yaw feedback, 125 mixed-command round trips");
}
'''
    with tempfile.TemporaryDirectory(prefix="vehicle-kinematics-test-") as temporary:
        directory = Path(temporary)
        cpp = directory / "check.cpp"
        binary = directory / "check"
        cpp.write_text(program)
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++14", "-Wall",
                        "-Wextra", "-Werror", str(cpp), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
