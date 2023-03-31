# MoveIt Studio Workspace

A sample workspace a MoveIt Studio User may have

## Changing the UR type in base_config.yaml

Replace ``ur5e`` with the model of your choice ``[ur3,ur3e,ur5,ur5e,ur10,ur10e,ur16e]``

## Build your workspace

``STUDIO_DOCKER_TAG=main docker compose up workspace_builder``

## Start MoveIt Studio

``docker compose up``

## Using a custom Docker overlay image

### Release image

To build:

``docker compose -f docker-compose-overlay.yaml build``

To start:

``docker compose -f docker-compose-overlay.yaml up``

### Developer container

We use Docker profiles to have separate configuration for developers.

To build:

``docker compose -f docker-compose-overlay.yaml build dev``

To start:

``docker compose -f docker-compose-overlay.yaml up dev``

And in a new Terminal, you can access a dev shell:

``docker compose exec -it dev bash``
