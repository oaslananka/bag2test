# CPU-only, opt-in ROS 2 integration environment; not part of fast PR checks.
FROM ros@sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698

ARG FAULT_INJECTION_REV=62a1e4bd9916eeb5ab270b21bdb5b10306cb203c
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       git libyaml-cpp-dev python3-venv \
       ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/fault_ws/src
RUN git clone https://github.com/reeceholland/ros2_fault_injection.git \
       ros2_fault_injection \
    && cd ros2_fault_injection \
    && git checkout --detach "$FAULT_INJECTION_REV" \
    && test "$(git rev-parse HEAD)" = "$FAULT_INJECTION_REV"

WORKDIR /workspaces/fault_ws
RUN bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build \
       --packages-select ros2_fault_injection \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --parallel-workers 4'

COPY pyproject.toml README.md LICENSE /opt/bag2test/
COPY src/ /opt/bag2test/src/
COPY tests/integration/runtime_probe.py /opt/bag2test/tests/integration/runtime_probe.py

RUN python3 -m venv --system-site-packages /opt/bag2test-venv \
    && /opt/bag2test-venv/bin/pip install --no-cache-dir /opt/bag2test

# Compile/install as root, execute only the synthetic test as an unprivileged user.
RUN useradd --create-home --uid 10001 --shell /usr/sbin/nologin bag2test
ENV HOME=/home/bag2test
USER bag2test

CMD ["/bin/bash","-lc","source /opt/ros/jazzy/setup.bash && source /workspaces/fault_ws/install/setup.bash && /opt/bag2test-venv/bin/python /opt/bag2test/tests/integration/runtime_probe.py"]
