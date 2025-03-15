FROM ubuntu:18.04

# non interactive flag to avoid getting stuck
ENV DEBIAN_FRONTEND=noninteractive 

# Make SSH configuration
RUN apt-get update && apt-get install -y \
    openssh-server \
    x11-apps \
    xauth \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /run/sshd

# Clean out any existing config
RUN echo "" > /etc/ssh/sshd_config

# Add fresh SSH configuration
RUN echo "Port 22" >> /etc/ssh/sshd_config && \
    echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo "PasswordAuthentication yes" >> /etc/ssh/sshd_config && \
    echo "X11Forwarding yes" >> /etc/ssh/sshd_config && \
    echo "X11UseLocalhost no" >> /etc/ssh/sshd_config

# Set a known password: "password"
RUN echo "root:password" | chpasswd


# Install dependencies
RUN  apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++ \
    gcc 

RUN apt-get update && apt-get install -y \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

EXPOSE 22

# Copy project files into container
# COPY . /app

# Run CMake and compile
# RUN mkdir build && cd build && cmake .. && make -j$(nproc)
# run ssh daemon
CMD ["/usr/sbin/sshd", "-D"]
