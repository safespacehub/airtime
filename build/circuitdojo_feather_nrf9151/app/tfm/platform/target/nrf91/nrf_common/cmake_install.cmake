# Install script for directory: /home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/turbo/.zephyrtools/toolchain/zephyr-sdk-0.16.4/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/interface/include/tfm_ioctl_core_api.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/interface/include" TYPE FILE MESSAGE_NEVER FILES "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/services/include/tfm_ioctl_core_api.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/interface/src/tfm_ioctl_core_ns_api.c")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/interface/src" TYPE FILE MESSAGE_NEVER FILES "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/services/src/tfm_ioctl_core_ns_api.c")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/config_nordic_nrf_spe.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core" TYPE FILE MESSAGE_NEVER FILES "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/config_nordic_nrf_spe.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/ext/driver/Driver_Common.h;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/ext/driver/Driver_Flash.h;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/ext/driver/Driver_USART.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/ext/driver" TYPE FILE MESSAGE_NEVER FILES
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/driver/Driver_Common.h"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/driver/Driver_Flash.h"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/driver/Driver_USART.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/startup.c;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/startup_nrf91.c;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/nrfx_glue.c;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/pal_plat_test.c;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/pal_plat_test.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core" TYPE FILE MESSAGE_NEVER FILES
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/startup.c"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/startup_nrf91.c"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/nrfx_glue.c"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/pal_plat_test.c"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/pal_plat_test.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/startup.h;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/target_cfg.h;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/nrfx_config.h;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/CMakeLists.txt;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/config.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core" TYPE FILE MESSAGE_NEVER FILES
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/startup.h"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/target_cfg.h"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/nrfx_config.h"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/ns/CMakeLists.txt"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/config.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/native_drivers;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/cmsis_drivers;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/common;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/nrfx;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/services;/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core/tests")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/common/core" TYPE DIRECTORY MESSAGE_NEVER FILES
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/native_drivers"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/cmsis_drivers"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/common"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/nrfx"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/services"
    "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/core/tests"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/linker_scripts/tfm_common_ns.ld")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/turbo/airtime2/app/build/circuitdojo_feather_nrf9151/app/tfm/api_ns/platform/linker_scripts" TYPE FILE MESSAGE_NEVER FILES "/home/turbo/airtime2/modules/tee/tf-m/trusted-firmware-m/platform/ext/common/gcc/tfm_common_ns.ld")
endif()

