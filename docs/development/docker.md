# Docker

## Installation

**For ubuntu, read [here](https://docs.docker.com/engine/install/ubuntu/).**

## Usage

### Development

1. Authenticate docker using `docker login` and your packages-enabled github token.
2. Build the image, e.g. `util/build`.
3. Tag the image, e.g. `docker tag svea ghcr.io/kth-sml/svea:latest`.
4. Push the image, `docker push ghcr.io/kth-sml/svea`.

