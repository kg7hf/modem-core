# SPDX-License-Identifier: GPL-3.0-or-later
# Download/verify the committed release without configuring a C++ compiler.
cmake_minimum_required(VERSION 3.25)
get_filename_component(source "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
include("${source}/cmake/ArticleData.cmake")
modem_article_pack("${source}/tests/data/first-path" "${source}/test-data")
