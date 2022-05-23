# CMakeList.txt: проект CMake для Console_Data_Logger_linux; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

add_executable (FileWriterExample FileWriterExample.cpp)

target_compile_features(FileWriterExample PRIVATE cxx_std_17)

target_link_libraries(FileWriterExample
    PRIVATE LMP_CommunicationLibrary)
