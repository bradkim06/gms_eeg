#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BOARD gms/nrf5340/cpuapp)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# ---- NCS v3.0.0 강제 ----
if(NOT DEFINED NCS_VERSION)
  message(FATAL_ERROR "nRF Connect SDK 환경이 감지되지 않았습니다. "
                      "이 프로젝트는 NCS v3.0.0 워크스페이스에서 빌드해야 합니다.")
endif()

if(NOT NCS_VERSION VERSION_EQUAL "3.0.0")
  message(FATAL_ERROR "이 프로젝트는 NCS v3.0.0 전용입니다. "
                      "현재 감지된 버전: v${NCS_VERSION} (NRF_DIR: ${NRF_DIR})")
endif()
message(STATUS "NCS 버전 확인 완료: v${NCS_VERSION}")
# -------------------------

project(GMS_EEG)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END
