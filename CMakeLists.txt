# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

# Bluedroid's default inquiry scan is an 11 ms listen every 1.28 s. Phones run
# long, repeated inquiries and eventually line up with that; macOS runs one
# shortish inquiry and gives up, so it never sees us. Listen 160 ms out of every
# 640 ms instead. These defaults are #ifndef-guarded in bt_target.h, so they can
# be overridden from here without touching IDF.
idf_build_set_property(COMPILE_DEFINITIONS "-DBTM_DEFAULT_DISC_WINDOW=0x0100" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DBTM_DEFAULT_DISC_INTERVAL=0x0400" APPEND)
project(a2dp_sink)
