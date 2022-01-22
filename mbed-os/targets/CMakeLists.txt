# Copyright (c) 2020-2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(${CMAKE_CROSSCOMPILING})
    include(mbed_set_linker_script)

    add_subdirectory(TARGET_STM EXCLUDE_FROM_ALL)

endif()
