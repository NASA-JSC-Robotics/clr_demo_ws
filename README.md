# Sample Containerized Workspace

This empty workspace can be used a starting point for a Docker-enabled VCS or Git Submodules workspace.
The contents of the `src` directory should be treated similarly to a "normal" ROS workspace.
That is, source code can be imported and added as needed to `src/`, then be built and run inside of an isolated, ROS enabled environment.

## Quick Development Setup

1) [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
    - Don't worry about Docker Desktop
    - For Ubuntu recommend using the [utility script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2) Fork or copy the contents of this repository as needed
3) Setup your source code for the `src/` directory
    - Either with git submodules (`git submodule add ...`)
    - Or with a repos file and vcs tool  (`vcs import ...`)
4) Set your user information for the project build
    - We recommend just putting this in your `~/.bashrc`:

      ```bash
      export USER_UID=$(id -u $USER)
      export USER_GID=$(id -g $USER)
      ```

    - Alternatively, open the `.env` file in the root of this repo and update each line with your information
        - `USER_UID` and `USER_GID`
            - found using `id -u` and `id -g` respectively

## Using the Images

Build the base images using the compose specification.

To build the development image from the repo root, and then launch it

```bash
# Compile the image
docker compose build

# Start it
docker compose up dev -d

# Connect to the console
docker compose exec dev bash
```

Once you're attached to the container, you can use it as a regular colcon workspace.
The contents of the `src/` directory will be mounted into `/home/er4-user/ws/src`.

### Other Things to Note

- Build logs, compiled artifaces, and the `.ccache` are also mounted in the workspace/user home.
This ensure artifacts are persisted even when restarting or recreating the container.

- The `.bash` folder gets mounted into your workspace, and the environment variable `HISTFILE` is set in the docker compose file.
This points the bash to keep the history in this folder, which will persist between docker container sessions so that your history is kept.

- Your host's DDS configuration (either cyclone or fastrtps) will be mounted into the image if set in your environment.
For more information refer to the [compose specification](docker-compose.yaml).

- Defaults for `colcon build` are set for the user. To change or modify, refer to the [defaults file](config/colcon-defaults.yaml).

- Two samples for GitLab CI for either [git submodules](.gitlab-ci.yml.submodules) or [vcs workspace](gitlab-ci.yml.vcs) are included.
Depending on your workflow, pick on and move it to `.gitlab-ci.yml` and it should build and push images, and run tests.
  - *NOTE:* There MUST be a `project.repos` file in the repo root to work with the VCS CI template.
  - *NOTE:* Images are tagged based on the repo's name/location and either the tag or branch of an MR.
