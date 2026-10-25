# SPDX-License-Identifier: GPL-3.0-or-later
# Tools only: mount the contributor's current checkout at /src.
FROM ubuntu:26.04@sha256:c098726dd9b9733f3e885f8de0ef375206a311b0a317eadc40d347b1115fb944
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates cmake ninja-build g++-14 clang-19 git \
    && rm -rf /var/lib/apt/lists/*
ENV CXX=g++-14
WORKDIR /src
ENTRYPOINT ["cmake"]
CMD ["-P", "tools/test.cmake"]
