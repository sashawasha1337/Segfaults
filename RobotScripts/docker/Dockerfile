FROM dustynv/ros:foxy-pytorch-l4t-r32.7.1
ENV DEBIAN_FRONTEND=noninteractive PIP_NO_CACHE_DIR=1 \
    MPLBACKEND=Agg

RUN rm -f /etc/apt/sources.list.d/ros2*.list && \
    apt-get update && apt-get install -y --no-install-recommends curl gnupg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2.list && \
    sed -i 's|http://archive.ubuntu.com|http://old-releases.ubuntu.com|g' /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git python3-dev python3-pip \
    python3-colcon-common-extensions python3-colcon-mixin python3-rosdep python3-vcstool \
    libgl1 libglib2.0-0 libsm6 libxrender1 libxext6 \
    software-properties-common wget xz-utils \
    libssl-dev libffi-dev zlib1g-dev \
 && rm -rf /var/lib/apt/lists/*

# rosdep

RUN rosdep init 2>/dev/null || true && rosdep update


# PyYAML fix for Jetson

RUN python3 -m pip install --upgrade "pip<21.4" "setuptools<60" "wheel<0.38"
RUN pip3 install --no-cache-dir --upgrade --ignore-installed "PyYAML==5.4.1"

#pip dependencies
RUN pip3 install --no-cache-dir \
    numpy==1.19.4 \
    pillow==8.4.0 \
    tqdm==4.64.1 \
    matplotlib==3.3.4 \
    seaborn==0.11.2 \
    "protobuf<4" \
    onnx==1.10.2 \
    thop==0.1.1.post2209072238 \
    filterpy==1.4.5 \
    cython==0.29.36 \
    lap==0.4.0


#python 3.9 for potential venv
WORKDIR /tmp
RUN wget https://www.python.org/ftp/python/3.9.18/Python-3.9.18.tgz && \
    tar -xf Python-3.9.18.tgz && \
    cd Python-3.9.18 && \
    ./configure --enable-optimizations && \
    make -j4 && make altinstall && \
    rm -rf /tmp/Python-3.9.18*


RUN /usr/local/bin/python3.9 -m venv /opt/py39-env
RUN /opt/py39-env/bin/pip install --upgrade pip setuptools wheel

# Firebase for venv
RUN /opt/py39-env/bin/pip install "firebase-admin<7.0.0" 


ENV PY39_VENV=/opt/py39-env/bin/python3.9
ENV SIDE_CAR_SCRIPT=/ros2_ws/src/venv_script.py


RUN git clone --depth 1 https://github.com/WongKinYiu/yolov7.git /ros2_ws/yolov7 && \
    sed -i '/opencv-python/d' /ros2_ws/yolov7/requirements.txt && \
        sed -i '/ipython/d' /ros2_ws/yolov7/requirements.txt

RUN sed -i 's/def check_requirements([^)]*):/def check_requirements(*args, **kwargs):\n    return/g' \
    /ros2_ws/yolov7/utils/general.py || true

ENV YOLOV5_SKIP_REQUIREMENTS=1


WORKDIR /ros2_ws
RUN mkdir -p src && cd src && \
    git clone -b foxy https://github.com/ros-perception/vision_opencv.git

#copy src goes here if doing that 

#COPY docker/workspace.sh ./workspace.sh
COPY docker/entrypoint.sh ./entrypoint.sh
RUN chmod +x /ros2_ws/*.sh


#venv stuff no longer necessary as camera functioned outside docker
#RUN mkdir -p /ros2_ws/src/scripts
#maybe we need to fix this addressing later?
#COPY src/camera/camera/venv_script.py /ros2_ws/src/scripts/venv_script.py
#RUN chmod +x /ros2_ws/src/scripts/venv_script.py

#debug
RUN echo "PY36:" && python3 --version && \
    echo "PY39:" && /opt/py39-env/bin/python3.9 --version

ENV OpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4



ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["bash"]

