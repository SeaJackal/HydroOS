FROM mcr.microsoft.com/devcontainers/base:jammy

ARG USERNAME=hydrolib
ARG UID=1001
ARG GID=$UID

# Install some dependencies packages
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - \
    && echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" \
    | sudo tee /etc/apt/sources.list.d/llvm.list \
    && apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    build-essential \
    gdb \
    cmake \
    libgtest-dev \
    doxygen \
    clang-format-19 \
    clangd-19 \
    openocd \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    gdb-multiarch \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-19 100 \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-19 100