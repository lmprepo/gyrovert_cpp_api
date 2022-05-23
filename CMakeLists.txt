# CMakeList.txt: файл проекта CMake верхнего уровня; проведите глобальную конфигурацию
# и включите сюда подпроекты.
#
cmake_minimum_required (VERSION 3.11)
project (Examples)

# Включите подпроекты.
add_subdirectory (LMP_CommunicationLibrary)

add_subdirectory (Example_1_ReceiveDataInConsole)

add_subdirectory (Example_2_ReadRawSensorsData)

add_subdirectory (Example_3_AlgorithmSelectionExample)


