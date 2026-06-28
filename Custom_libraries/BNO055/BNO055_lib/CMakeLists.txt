# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(BNO055_lib_exp)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/bno055_driver.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(.)
