# Imagen del demo RoboCup para el simulador de Booster Studio (1 vs nadie).
# Construir desde la raiz del repo:
#   docker build -f sim_docker/Dockerfile -t robocup-demo-sim .
ARG BASE_IMAGE=booster-robotics-registry.cn-beijing.cr.aliyuncs.com/virtual-robot/virtual-robot:0.9.4-alpha-humble
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git dos2unix \
        libopencv-dev libpcl-dev libgoogle-glog-dev libgflags-dev \
        libyaml-cpp-dev libeigen3-dev nlohmann-json3-dev \
        python3-colcon-common-extensions \
        ros-humble-behaviortree-cpp ros-humble-backward-ros \
        ros-humble-image-transport ros-humble-cv-bridge \
        ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
        ros-humble-pcl-conversions ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# ONNX Runtime (CPU) desde el tarball del repo
COPY third_party/onnxruntime-linux-x64-1.23.2.tgz /tmp/
RUN tar -xzf /tmp/onnxruntime-linux-x64-1.23.2.tgz -C /tmp \
    && cp -r /tmp/onnxruntime-linux-x64-1.23.2/include/* /usr/local/include/ \
    && cp -r /tmp/onnxruntime-linux-x64-1.23.2/lib/* /usr/local/lib/ \
    && ldconfig && rm -rf /tmp/onnxruntime*

# Booster Robotics SDK (cabeceras y libs) que trae la imagen del robot virtual
RUN HDR=$(find / -path /proc -prune -o -name b1_api_const.hpp -print 2>/dev/null | head -1); \
    SDK=${HDR%/robot/b1/b1_api_const.hpp}; \
    echo "SDK include: ${SDK}"; \
    if [ -n "$SDK" ] && [ "$SDK" != "/usr/local/include/booster" ]; then \
        cp -rn "$SDK" /usr/local/include/; \
        LIBDIR=$(dirname $(dirname "$SDK"))/lib; \
        [ -d "$LIBDIR" ] && cp -rn "$LIBDIR"/* /usr/local/lib/ || true; \
    fi; \
    ldconfig; ls /usr/local/include/booster/robot/b1/b1_api_const.hpp

# Copia del repo (CRLF -> LF solo en la copia del contenedor)
COPY . /opt/robocup_demo
WORKDIR /opt/robocup_demo
RUN find . -type f \( -name '*.sh' -o -name '*.py' -o -name '*.yaml' -o -name '*.xml' \
        -o -name '*.txt' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.msg' -o -name '*.srv' \) \
        -print0 | xargs -0 dos2unix -q

ARG BUILD_JOBS=4
RUN source /opt/ros/humble/setup.bash \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --base-paths src \
        --parallel-workers 2 \
        --packages-skip detection_converter \
        --cmake-args -DNO_CUDA=ON -DCMAKE_BUILD_TYPE=Release

COPY sim_docker/run_demo.sh /opt/run_demo.sh
RUN dos2unix -q /opt/run_demo.sh && chmod +x /opt/run_demo.sh

ENTRYPOINT ["/opt/run_demo.sh"]
