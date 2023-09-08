# Docker

This project is using [Docker](https://www.docker.com/) to containerize the software. We host
container images on [GitHub Packages](https://github.com/features/packages).

## Motivation

Previously all SVEA images were built when you wanted to use them. This could take a long time even
for smaller projects. The reason was that each and every build had to download and install all
dependencies, even though many were a core dependency. Ideally we wanted an base image that was
prebuilt for any SVEA project. GitHub Packages allows us to do exactly that.

## Usage

Currently we have two Dockerfiles (this can come to change if we have special images for e.g.
Stereolabs ZED), `docker/Dockerfile.base` for the base build and `docker/Dockerfile` that end-users
will build from. Any end-user that has a SVEA project should simply call `util/build` as is
described in the tutorials. This will invoke `docker build` on `docker/Dockerfile` with base image
`ghcr.io/kth-sml/svea`. `ghcr.io` is the Github Container Registry (similar to Docker Hub). The
end-user's `docker/Dockerfile` should copy in any new ROS code and call `rosdep`, `catkin build` and
the like to build her project. However, they will benefit from a prebuilt SVEA workspace and the
installation  will take shorter than otherwise.

On the other hand we must also maintain SVEA. A maintainer of SVEA should carefully consider all
workflows with and around docker, and be familiar with the [Container Registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry).
If a maintainer wants to update SVEA images on `ghcr.io` then the following should be done.

1. Provide the `docker` CLI a GitHub token (Read in link above).
2. Build SVEA with the `BUILD_CONFIG` option for all different targets.
  - On a x86 computer, `BUILD_CONFIG=base util/build`
  - On a x86 computer, `BUILD_CONFIG=desktop util/build`
  - On a jetson computer, `BUILD_CONFIG=base util/build`
3. Tag the latest SVEA image.
  - `docker tag ghcr.io/kth-sml/svea:x86_64-base ghcr.io/kth-sml/svea`
4. Publish to the Container Repository.
  - `docker push ghcr.io/kth-sml/svea`

## Future

### util/config.sh

We need to make sure that it is easy for the end-user to build and run SVEA. At the same time it
should work on x86 or Jetson, windows or linux, ROS core installation or desktop, etc. This is
possible today but may require janky solutions. This is probably something that *might* become
easier with ROS 2.

### Cross Compilation

With buildkit it's possible to cross compile on a x86 computer to other architects. This could/will
significantly improve effort to maintain the Github Container Registry. Ideally there would be a
shell script or something that automates all builds and publishes them to `ghcr.io`.
