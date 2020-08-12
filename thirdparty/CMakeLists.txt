#Author: Jan Brejcha
#project: AdobeTrips

include_directories(cpp_exiftool/inc)
add_library(cpp_exiftool cpp_exiftool/src/ExifTool.cpp cpp_exiftool/src/ExifToolPipe.cpp cpp_exiftool/src/TagInfo.cpp)

include_directories(nanoflann/include)
include_directories(json/src)
