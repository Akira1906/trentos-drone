#
# Demo TUM Course
#
# Copyright (C) 2021, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.7.2)

#-------------------------------------------------------------------------------
os_set_config_file(
    "system_config.h"
    CONFIG_PROJECT_NAME "system_config"
)

#-------------------------------------------------------------------------------
project(demo_tum_course C)

include("plat/${PLATFORM}/plat_nic.cmake")

TimeServer_DeclareCAmkESComponent(
    TimeServer
)

RPi_SPI_Flash_DeclareCAmkESComponent(
    RPi_SPI_Flash
    LIBS
        TimeServer_client
)

StorageServer_DeclareCAmkESComponent(
    StorageServer
)

NetworkStack_PicoTcp_DeclareCAmkESComponent(
    NetworkStack_PicoTcp
    C_FLAGS
        -DNetworkStack_PicoTcp_USE_HARDCODED_IPADDR
        -DDEV_ADDR=ETH_ADDR
        -DGATEWAY_ADDR=ETH_GATEWAY_ADDR
        -DSUBNET_MASK=ETH_SUBNET_MASK
)

DeclareCAmkESComponent(
    TestApp
    SOURCES
        components/TestApp/src/TestApp.c
        components/TestApp/src/utils.c
        components/TestApp/src/communication.c
    C_FLAGS
        -Wall
        -Werror
    LIBS
        system_config
        lib_debug
        os_core_api
        os_filesystem
        os_socket_client
)

DeclareAndCreateCamkESSystem(demo_tum_course.camkes)
GenerateSimulateScript()