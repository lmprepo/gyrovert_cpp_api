# CMakeList.txt: проект CMake для Receive_ADC_Data_Example; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)


add_executable (ReadRawSensorsData ReadRawSensorsData.cpp)

target_link_libraries(ReadRawSensorsData PRIVATE LMP_CommunicationLibrary)