# Maintaining Docker Images

There are two Dockerfiles:

- `docker/Dockerfile.base` installs ROS 2 Jazzy dependencies, PX4 message
  definitions, Better Launch, and DISCOWER's Qualisys driver under
  `/opt/svea/jazzy`. It then prepares the SVEA workspace.
- `docker/Dockerfile` builds the checked-out workspace on top of
  `ghcr.io/kth-sml/svea:latest`, installs its dependencies, and uses colcon.

End users run `util/build` and `util/run`; see the
[development workflow](../setup/docker.md).

## Build configuration

Inspect `util/config.sh` and the generated commands before building:

```bash
DEBUG=1 util/build
DEBUG=1 BUILD_CONFIG=base util/build
DEBUG=1 BUILD_CONFIG=ghcr util/build
```

For a local base image, choose a separate tag so you do not confuse it with a
published release:

```bash
BUILD_CONFIG=base IMAGE_TAG=svea-base-local util/build
BUILD_TAG=svea-base-local util/build
```

`base-amd64` and `base-arm64` select a specific base-image architecture. The
`ghcr` configuration builds for both `linux/arm64` and `linux/amd64` and pushes
to GHCR, so it requires registry credentials, a configured buildx builder,
and cross-platform build support. Do not use it for an ordinary local build.

The `.github/workflows/update-ghcr.yml` workflow logs into GHCR and runs this
publishing configuration. Its repository guard limits publication to
`KTH-SML/svea`. Review the workflow and `util/config.sh` before a release.

The base Dockerfile currently leaves Zenoh bridge installation disabled.
Publishing port 7447 does not install or start a bridge, and `use_zenoh=True`
requires `zenoh-bridge-ros2dds` to be installed separately.
