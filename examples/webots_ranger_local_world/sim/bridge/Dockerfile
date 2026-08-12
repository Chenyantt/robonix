# SPDX-License-Identifier: MulanPSL-2.0
# ROS 2 Humble + Webots (GUI) + Nav2 deps + Python deps for robonix
# drivers that get `docker exec`'d into this container by `rbnx boot`.
# This image is the SIM container; it does NOT contain robonix driver
# code. Driver code lives in each primitive/service package directory
# on the host and is bind-mounted in by compose.yaml under /robonix_pkgs.
# Build context = the directory containing this Dockerfile (sim/bridge/).
ARG ROS_BASE_IMAGE=robonix_tiago_sim-sim:latest
FROM ${ROS_BASE_IMAGE}

# ── TUNA mirrors ───────────────────────────────────────────────────
# Swap apt sources (Ubuntu/ports + ROS 2) to TUNA. Use `find` + sed
# --follow-symlinks because /etc/apt/sources.list.d/ros2.sources is
# a symlink into /usr/share/ros-apt-source/ in the upstream ros image.
RUN set -eux; \
    find /etc/apt/sources.list.d/ -maxdepth 1 \( -name '*.list' -o -name '*.sources' \) -print0 \
      | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
    if [ -f /etc/apt/sources.list ]; then \
        sed -i --follow-symlinks \
            -e 's|http://archive\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://security\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://ports\.ubuntu\.com/ubuntu-ports/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|g' \
            /etc/apt/sources.list; \
    fi
ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    UV_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    HF_ENDPOINT=https://hf-mirror.com
ENV DEBIAN_FRONTEND=noninteractive

# xvfb / Xorg / mesa-utils / pciutils are needed by the headless display
# backends (`WEBOTS_HEADLESS_MODE` in entrypoint.sh). They are tiny and
# the legacy host-DISPLAY path doesn't use them, so keeping them always
# installed avoids a second image variant.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libxcb-cursor0 \
    xvfb \
    xserver-xorg-core \
    mesa-utils \
    pciutils \
    python3-pip \
    python3-colcon-common-extensions \
    unzip \
    ros-humble-nav2-msgs \
    ros-humble-nav2-bringup \
    ros-humble-webots-ros2 \
    ros-humble-webots-ros2-driver \
    ros-humble-webots-ros2-control \
    ros-humble-controller-manager \
    ros-humble-controller-interface \
    ros-humble-diagnostic-updater \
    ros-humble-diff-drive-controller \
    ros-humble-hardware-interface \
    ros-humble-joint-state-broadcaster \
    ros-humble-realtime-tools \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    wget \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# This local Ranger example intentionally layers on top of the already-built
# Webots sim image so it can rebuild the robot workspace without re-downloading
# the large Cyberbotics .deb.

# Keep the remote viewer in read-only W3D mode by default. The public stream
# port is injected by viewer_server.py at runtime so custom ports work with
# both host and bridge networking. Webots' raw stream remains on port 1234.
RUN sed -i \
      -e 's#https://cyberbotics.com/wwi/R2025a/WebotsView.js#/wwi-full/WebotsView.js#' \
      -e 's#</body>#<script>const wsPort=new URLSearchParams(location.search).get("wsPort")||"__WEBOTS_STREAM_PORT__";document.getElementById("IP-input").value="ws://"+location.hostname+":"+wsPort;document.getElementById("mode").value="w3d";document.getElementById("broadcast").checked=false;</script></body>#' \
    /usr/local/webots/resources/web/streaming_viewer/index.html

# Python deps used by the robonix driver processes that will be `docker
# exec`'d into this container. (Drivers themselves live on host; their
# Python imports resolve against this image's site-packages + the
# bind-mounted package dirs under /robonix_pkgs.)
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel
RUN python3 -m pip install --no-cache-dir \
    "grpcio>=1.78.0" "protobuf>=6.30,<7" mcp "fastmcp>=3" numpy Pillow uvicorn httpx \
    "websockets>=14,<17"

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Build context is sim/bridge/ — ros_ws lives one level up at sim/ros_ws/.
# Compose passes context: ../ so we COPY from there.
RUN rm -rf /colcon_ws
WORKDIR /colcon_ws
COPY ros_ws/src src
RUN bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select four_wheel_steering_controller ranger_mini_v3 eaios_webots"

# In WSL2, Docker containers inherit a Microsoft kernel string. The Webots
# ROS 2 driver should still treat the container as Linux; otherwise it tries
# Windows/WSL path and networking rules and the Webots controllers never attach.
RUN sed -i \
      "s/return 'microsoft-standard' in uname().release/return 'microsoft-standard' in uname().release and not is_docker()/" \
    /opt/ros/humble/local/lib/python3.10/dist-packages/webots_ros2_driver/utils.py

COPY bridge/entrypoint.sh /entrypoint.sh
COPY bridge/viewer_server.py /viewer_server.py
COPY bridge/webots_stream_proxy.py /webots_stream_proxy.py
COPY bridge/streaming_healthcheck.py /streaming_healthcheck.py
RUN chmod +x /entrypoint.sh /viewer_server.py /webots_stream_proxy.py /streaming_healthcheck.py

WORKDIR /
ENTRYPOINT ["/entrypoint.sh"]
