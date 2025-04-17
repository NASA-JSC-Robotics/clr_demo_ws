#!/bin/bash

# For pip installed packages
export PATH="${HOME}/.local/bin:${PATH}"

# This change is required to support loading pyassimp on arm64 machines.
if [ "$(uname -m)" = "aarch64" ]; then \
    echo "Adding /usr/lib/aarch64-linux-gnu to linker's library path for arm64 support"
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu; \
fi

# Source the ROS install
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source the ROS 2 workspace install, if it exists
if [ -f "${ER4_WS}/install/setup.bash" ]; then
    echo "Sourcing ${ER4_WS}/install/setup.bash"
    # shellcheck source=/dev/null
    source "${ER4_WS}/install/setup.bash"
else
    echo "The ${ER4_WS} workspace is not yet built."
    echo "To build:"
    echo "  cd ${ER4_WS}"
    echo "  colcon build"
fi

# Pass through
exec "$@"
