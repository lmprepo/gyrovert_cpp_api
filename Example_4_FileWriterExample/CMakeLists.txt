# CMakeList.txt: проект CMake для Console_Data_Logger_linux; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)
find_package(Threads REQUIRED)

add_library(GKV_FileWriter
    GKV_FileWriter.cpp
    GKV_FileWriter.h)

    target_compile_features(GKV_FileWriter PRIVATE cxx_std_17)



    target_link_libraries(GKV_FileWriter PRIVATE LMP_CommunicationLibrary)

    target_include_directories(GKV_FileWriter PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable (FileWriterExample FileWriterExample.cpp)

target_compile_features(FileWriterExample PRIVATE cxx_std_17)

target_link_libraries(FileWriterExample
    PRIVATE LMP_CommunicationLibrary)

target_link_libraries(FileWriterExample
    PRIVATE GKV_FileWriter)


