# SPDX-License-Identifier: GPL-3.0-or-later
# Tools only: mount the contributor's current checkout at /src.
FROM ubuntu:24.04@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates cmake ninja-build g++-14 clang-19 git \
    && rm -rf /var/lib/apt/lists/*
ENV CXX=g++-14
WORKDIR /src
ENTRYPOINT ["cmake"]
CMD ["-P", "tools/test.cmake"]
