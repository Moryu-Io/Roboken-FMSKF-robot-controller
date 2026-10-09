#!/usr/bin/env bash
# Add one message to the bundled 2.0.7-humble archive without replacing its RMW.
set -euo pipefail
library_root=$(cd "$(dirname "$0")/../.." && pwd)
work=${1:-$(mktemp -d /tmp/arm-progress-microros.XXXXXX)}
mkdir -p "$work"
work=$(cd "$work" && pwd)
cp -a "$library_root/extras/library_generation" "$work/"
image=${MICROROS_BUILDER_IMAGE:-sha256:bab8476bc71f52332596f2ce15417696ce8ecfbd5a4d3b569c716a5bd34327a4}
docker run --rm --entrypoint bash -v "$work:/work" "$image" -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source /uros_ws/install/local_setup.bash
  cd /work
  if [ ! -d firmware ]; then
    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
  fi
  rm -rf firmware/mcu_ws/interfaces
  cp -a library_generation/extra_packages/interfaces firmware/mcu_ws/interfaces
  export TOOLCHAIN_PREFIX=/uros_ws/teensy-compile/tools/arm/bin/arm-none-eabi-
  ros2 run micro_ros_setup build_firmware.sh /work/library_generation/teensy4_toolchain.cmake /work/library_generation/colcon.meta
  chmod -R a+rwX /work/firmware
'
python3 "$library_root/extras/library_generation/install_arm_progress_type.py" "$work/firmware/build" "$library_root"
printf 'Generated ArmExecutionProgress in %s\n' "$work"
