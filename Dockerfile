FROM ubuntu:16.04

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

EXPOSE 22

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++-5 \
    gcc-5 \
    libpcl-dev=1.7.2-* \
    && rm -rf /var/lib/apt/lists/*

# Set GCC 5 as default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 100

# Set working directory
WORKDIR /app

# Copy project files into container
COPY . /app

# Run CMake and compile
RUN mkdir build && cd build && cmake .. && make -j$(nproc)
# run ssh daemon
CMD ["/usr/sbin/sshd", "-D"]
