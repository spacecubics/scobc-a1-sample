# Copyright (c) 2025 Space Cubics Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello-world)

target_sources(app PRIVATE src/main.c)
