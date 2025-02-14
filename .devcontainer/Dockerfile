# FROM isaac_ros_dev-aarch64
# FROM ros:humble
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ARG CONDA_VER=latest
ARG OS_TYPE=x86_64
SHELL ["/bin/bash", "-c"]

# uodate and install dependencies 
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
    libyaml-cpp-dev \
    # libboost-all-dev\
    cmake-qt-gui \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libxext6 \
    libx11-6 \
    freeglut3-dev \
    git \
    python3-pip \
    python3-tk \
    curl \
    vim \ 
    libcgal-dev \
    libcgal-demo \
    python-is-python3 \
    build-essential \
    cmake \
    libeigen3-dev \
    python3-dev \
    libglm-dev \
    wget \
    ninja-build \
    fzf \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Miniconda
RUN wget https://github.com/conda-forge/miniforge/releases/${CONDA_VER}/download/Miniforge3-Linux-${OS_TYPE}.sh -O ~/miniconda.sh \
    && /bin/bash ~/miniconda.sh -b -p /opt/conda \
    && rm ~/miniconda.sh \
    && ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh \
    && echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc \
    && echo "conda activate base" >> ~/.bashrc

ENV PATH /opt/conda/bin:$PATH  
# SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
RUN conda create -n fingerpy python=3.10
SHELL ["conda", "run", "-n", "fingerpy", "/bin/bash", "-c"]

ENV CONDA_PREFIX /opt/conda
ENV CYCLONEDDS_HOME $CONDA_PREFIX

# Install the dependencies

# pinocchio and crocoddyl
RUN conda run -n fingerpy conda install -y -c conda-forge \
                        pinocchio \
                        crocoddyl \
                        cxx-compiler \
                        cmake \ 
                        jupyter \
                        ipykernel \
                        yaml-cpp \
                        opencv



RUN pip install mujoco \
                pyyaml \
                matplotlib \
                rerun-sdk \
                opencv-python \
                opencv-contrib-python \
                imageio[ffmpeg] \
                pyrealsense2 \
                pygame \
                meshcat



# # CycloneDDS
WORKDIR /root
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds && \
    cd cyclonedds && mkdir build install && cd build && \ 
    cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX && \
    cmake --build . --config RelWithDebInfo --target install 

RUN echo "export CYCLONEDDS_HOME=$CONDA_PREFIX" >> ~/.bashrc
RUN pip3 install git+https://github.com/eclipse-cyclonedds/cyclonedds-python

# CycloneDDS-cxx
WORKDIR /root
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx &&\
    cd cyclonedds-cxx && mkdir build install && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX \
        -DCMAKE_PREFIX_PATH=$CONDA_PREFIX .. && \
    cmake --build . --target install
    

# RUN conda install -n fingerpy  
RUN conda run -n fingerpy python -m ipykernel install --name "fingerpy" --display-name "Python (fingerpy)"
# RUN python -m ipykernel install --name "fingerpy" --display-name "Python (fingerpy)"
RUN echo "conda activate fingerpy" >> ~/.bashrc
# Env vars for the nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

ENTRYPOINT [ "/bin/bash", "-l", "-c" ]