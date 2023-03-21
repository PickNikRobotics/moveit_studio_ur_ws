# MoveIt Studio Workspace

A sample workspace a MoveIt Studio User may have

## Changing the UR type in base_config.yaml

Replace the ``ur16e`` with the model of your choice ``[ur3,ur3e,ur5,ur5e,ur10,ur10e,ur16e]``

## Build your workspace

``STUDIO_DOCKER_TAG=main docker compose up workspace_builder``

## Start MoveIt Studio

``docker compose up``