#!/usr/bin/env bash
set -euo pipefail
root=/opt/solo/build-inputs/apt
mkdir -p "$root/debs" "$root/lists" "$root/sources"
cp -a /var/cache/apt/archives/*.deb "$root/debs/"
cp -a /var/lib/apt/lists/. "$root/lists/"
cp -a /etc/apt/sources.list.d/. "$root/sources/"
(cd "$root" && dpkg-scanpackages --multiversion debs /dev/null > Packages)
(cd "$root" && sha256sum debs/*.deb > SHA256SUMS)
dpkg-query -W -f='${binary:Package}=${Version}\n' | sort > "$root/installed.lock"
