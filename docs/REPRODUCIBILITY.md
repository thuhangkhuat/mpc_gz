# Reproducible deployment

[Project overview](../README.md) · [Validation record](VALIDATION.md)

Use [compose.release.yaml](../compose.release.yaml) to run tested, digest-addressed
images without a checkout. Installed code comes from `/opt/solo_ws/install`,
configuration is a read-only input, and output/temporary directories are writable.
Preserve the platform, image digest, and input snapshots together.

Development [compose.yaml](../compose.yaml) sets `SOLO_DEV=1` and imports live
checkout modules. Its image digest identifies dependencies; source hashes are
also needed to identify the application that ran.

## Deploy a tested image

Get the **runtime** registry digest for your architecture from the
[release-images workflow](../.github/workflows/release.yml) acceptance artifact or
job summary. Input/archive digests serve a different purpose. The workflow builds
and tests on native AMD64/ARM64 runners, publishing each passing platform.
Companion smoke acceptance does not establish FCU connectivity or flight support.

In an empty deployment directory, with Docker Compose and Python 3 available:

```bash
mkdir solo-release
cd solo-release
export IMAGE='ghcr.io/OWNER/solo-simulation@sha256:ACTUAL_PLATFORM_DIGEST'
docker pull "$IMAGE"
extract=$(docker create --entrypoint /bin/true "$IMAGE")
docker cp "$extract:/opt/solo/deploy/." .
docker cp "$extract:/workspace/config" config
docker rm "$extract"
cp .env.release.example .env
mkdir -p runs artifacts
```

Edit `.env` before continuing:

- Set both `SIM_IMAGE` and `COMPANION_IMAGE` to tested registry digests. Compose
  requires both variables even when starting one service.
- Set `RELEASE_PLATFORM` to `linux/amd64` or `linux/arm64`, matching those images.
- Set `LOCAL_UID`/`LOCAL_GID` to the owner of the output directories.
- Keep release paths, or copy custom scenario/controller YAML under `config/`
  and select it with `/workspace/config/...` paths. Do not reuse development
  `.env` paths pointing into a checkout.

Then pull and start:

```bash
./release pull
./release doctor
./release up sim
./release logs -f sim
```

The helper rejects tags and unsupported platforms. The filesystem/configuration
are read-only; `/tmp` is tmpfs for HOME, Gazebo, and JIT caches. Bind directories
must exist and be writable by the configured user. Compose does not create them.

Wait for fleet and node readiness in the logs, then explicitly fly and clean up:

```bash
./release doctor
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc mission run --seconds 20
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc evaluate
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc mission stop
./release stop sim
```

Confirm mission cleanup succeeds before stopping the service. The `release`
helper's `stop` passes directly to Compose; it does not request LAND itself.
`doctor` can verify the live server renderer after startup.

For GPU/display/serial overrides, use
`docker compose --env-file .env -f compose.release.yaml -f <override> ...`
consistently, with actual device group IDs. The helper always selects the standalone
headless base. See [GPU setup](WSL_GPU.md) and [hardware migration](HARDWARE.md).

## What is preserved

| Input | Preservation |
|---|---|
| ROS base / uv | Dockerfile digest pins and [versions.json](../locks/versions.json) |
| ArduPilot, MAVROS, Gazebo plugin | Exact commits and ArduPilot submodule manifest |
| Python environments | Hash-locked `locks/*lock`, downloaded cache, build environments |
| GeographicLib | Verified [payload hashes](../locks/geographiclib.sha256) |
| Debian packages | Downloaded `.deb` files, checksums, indexes, repository metadata |
| Application and build | Source snapshot, upstream Git trees, build arguments, compiler outputs |

APT discovery uses mutable repositories, so a new build may resolve new patches
and needs fresh validation. Runtime stages install only the exact archived `.deb`
set without repository access. Changed GeographicLib payloads fail hash checks.

Runtime images contain ROS installs, SITL binaries/defaults, plugins/models,
numerical libraries, datasets, and notices. Build trees, compiler tools, headers,
static/object libraries, uv, and the Python 3.11 build environment are excluded.
Flattening removes deleted build material from lower layers.
`removed-build-files.lock` records pruning; `apt-packages.lock` records package
versions before pruning. Updating APT inside a pruned image is unsupported;
rebuild the release.

The workflow saves checksummed runtime/input archives, tests on fresh native
runners with no checkout, then publishes runtime, input, and clean-base images.
Acceptance records their registry digests and platform. CI artifacts expire
after 90 days: archive acceptance evidence and retain input/base images for the
supported lifetime of a release.

## Build and smoke-test a candidate

From the repository root:

```bash
docker build --target simulation_mpc -t solo-simulation:candidate -f docker/Dockerfile .
docker build --target simulation_inputs -t solo-simulation-inputs:candidate -f docker/Dockerfile .
./scripts/release-smoke solo-simulation:candidate sim artifacts/release-smoke
docker build --target base_inputs -t solo-base:inputs -f docker/Dockerfile .
docker save solo-simulation:candidate solo-simulation-inputs:candidate solo-base:inputs -o release.tar
sha256sum release.tar
```

For companion builds, use targets `companion` / `companion_inputs` and smoke
service `companion`. Smoke extracts deployment files, checks installed imports,
MAVROS overlay, application/dataset hashes, shared libraries, filesystem writes,
and absence of build tools. Simulation also requires three FCU connections,
fresh poses, initialized nodes, and the dedicated server renderer log. It never
arms. Evidence remains in the requested directory.

## Replay preserved inputs

Use the matching archived source snapshot, including its Dockerfile, as context:

```bash
docker pull ghcr.io/OWNER/solo-simulation-inputs@sha256:INPUT_DIGEST
inputs='docker-image://ghcr.io/OWNER/solo-simulation-inputs@sha256:INPUT_DIGEST'
docker build --network=none \
  --build-arg ROS_BASE=ghcr.io/OWNER/solo-simulation-base-inputs@sha256:BASE_DIGEST \
  --build-context common_build="$inputs" \
  --build-context simulation_dependencies="$inputs" \
  --build-context simulation_mpc_dependencies="$inputs" \
  --target simulation_mpc -t solo-simulation:replayed -f docker/Dockerfile .
```

This reuses preserved compiler outputs; it does not promise bit-for-bit upstream
recompilation. For exact deployment, pull the tested runtime digest or load its
checksummed archive. With local archives, use `ROS_BASE=solo-base:inputs`.

## Identify an experiment

Development runs record live source hashes. Release runs copy the image's
`application.sha256` alongside configuration snapshots, package locks, and dataset
checksums. `./scripts/dev lock` exports manifests and image identity to `artifacts/`;
also record a registry digest when sharing. See [run outputs](OPERATIONS.md#logs-and-evaluation).

Dependency changes require regenerated hashes, a rebuilt candidate, and native
acceptance. AMD64 emulation/build success does not validate ARM64 runtime. Physical
deployment still requires the [hardware validation stages](HARDWARE.md#6-validate-in-stages).
