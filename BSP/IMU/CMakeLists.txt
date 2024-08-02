cmake_minimum_required(VERSION 3.22)

project(bsp_imu)

add_library(bsp_imu STATIC
    wit_c_sdk/wit_c_sdk.c
    # imu_thread.c
)

target_include_directories(bsp_imu PUBLIC
    wit_c_sdk
)
