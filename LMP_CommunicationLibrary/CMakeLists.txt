cmake_minimum_required(VERSION 3.11)
find_package(Threads REQUIRED)

add_library(LMP_CommunicationLibrary
    src/LMP_Device.cpp
    src/GKV_Device.h
    src/LMP_AlgParamPacket.h
    src/LMP_AlorithmPackets.h
    src/LMP_BasePacket.h
    src/LMP_Device.h
    src/LMP_CustomPacket.h
    src/LMP_FilterPacket.h
    src/LMP_IfProto.h
    src/LMP_GyroOffsetPacket.h
    src/LMP_SettingsPacket.h
    src/LMP_TestPacket.h)

    target_compile_features(LMP_CommunicationLibrary PRIVATE cxx_std_17)

    target_link_libraries(LMP_CommunicationLibrary ${CMAKE_THREAD_LIBS_INIT})

    target_include_directories(LMP_CommunicationLibrary PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
