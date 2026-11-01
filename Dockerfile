# SPDX-License-Identifier: GPL-3.0-or-later
# Tools only: mount the contributor's current checkout at /src.
FROM ubuntu:26.04@sha256:6a06111c3b0a8600feadb270aec771dc239bf407b25625f93f44ab00e11f4d3c
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates cmake ninja-build g++-14 clang-19 git \
    && rm -rf /var/lib/apt/lists/*
ENV CXX=g++-14
WORKDIR /src
ENTRYPOINT ["cmake"]
CMD ["-P", "tools/test.cmake"]
