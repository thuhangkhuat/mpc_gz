#!/usr/bin/env bash
set -euo pipefail
# ROS Debian runtime dependencies include development packages. Removing those
# packages through APT also removes ROS messages/CLI. Remove their build-only
# files explicitly, retain shared libraries, package resources and all copyright
# files, and record the removals alongside the installed-package manifest.
mkdir -p /opt/solo
find /usr/bin /usr/lib /usr/share /usr/include /opt/ros/jazzy \
  \( -path /usr/include -o -path /usr/lib/gcc -o -path '/usr/share/cmake*' \
     -o -path '/usr/share/colcon*' -o -path '/usr/share/doc' \) -prune -o \
  -type f \( -name '*.a' -o -name '*.o' \) -print > /opt/solo/removed-build-files.lock
find /usr/bin -maxdepth 1 \( -name '*gcc*' -o -name '*g++*' -o -name '*c++*' \
  -o -name '*cpp*' -o -name 'cc' -o -name 'make' -o -name 'cmake' \
  -o -name 'ctest' -o -name 'cpack' -o -name 'colcon' \) -print \
  >> /opt/solo/removed-build-files.lock
xargs -r rm -f < /opt/solo/removed-build-files.lock
for path in /usr/include /usr/lib/gcc /usr/share/cmake* /usr/share/colcon*; do
  printf '%s\n' "$path" >> /opt/solo/removed-build-files.lock
  rm -rf "$path"
done
find /opt/ros/jazzy /opt/mavros_ws/install /opt/solo_ws/install \
  -type d \( -name include -o -name cmake \) -prune -print \
  >> /opt/solo/removed-build-files.lock
find /opt/ros/jazzy /opt/mavros_ws/install /opt/solo_ws/install \
  -type d \( -name include -o -name cmake \) -prune -exec rm -rf {} +
find /usr/lib/python3/dist-packages -maxdepth 1 -name 'colcon*' -print \
  >> /opt/solo/removed-build-files.lock
find /usr/lib/python3/dist-packages -maxdepth 1 -name 'colcon*' -exec rm -rf {} +
# Keep all dependency license / notice files under /usr/share/doc and site-packages.
dpkg-query -W -f='${binary:Package}=${Version}\n' | sort > /opt/solo/apt-packages.lock
find /opt/solo_ws/install /workspace/scripts /opt/solo/deploy /usr/local/bin/solo-entrypoint -type f ! -name '*.pyc' -print0 | sort -z | \
  xargs -0 sha256sum > /opt/solo/application.sha256
rm -f /usr/local/bin/install-runtime /usr/local/bin/prune-runtime
