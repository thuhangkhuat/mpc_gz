#!/usr/bin/env bash
set -euo pipefail
# /inputs is a read-only BuildKit mount from the corresponding compiler stage.
(cd /inputs && sha256sum -c SHA256SUMS >/dev/null)
rm -f /etc/apt/sources.list /etc/apt/sources.list.d/*
printf 'deb [trusted=yes] file:/inputs ./\n' > /etc/apt/sources.list.d/release.list
apt-get update --error-on=any
apt-get install -y --no-install-recommends "$@"
rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
rm /etc/apt/sources.list.d/release.list
