# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/doannhat214031/esp-idf/components/bootloader/subproject"
  "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader"
  "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader-prefix"
  "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader-prefix/tmp"
  "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader-prefix/src/bootloader-stamp"
  "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader-prefix/src"
  "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/doannhat214031/Bachelor-of-Science-Research-Project/source_code/gatt_server/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
