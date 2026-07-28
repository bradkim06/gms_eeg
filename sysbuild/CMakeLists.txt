# 프로젝트 루트가 boards/ 디렉터리를 포함한다.
get_filename_component(APP_BOARD_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

list(APPEND BOARD_ROOT "${APP_BOARD_ROOT}")

find_package(Sysbuild REQUIRED HINTS "$ENV{ZEPHYR_BASE}")

project(sysbuild LANGUAGES)
