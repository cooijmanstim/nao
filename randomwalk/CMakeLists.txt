cmake_minimum_required(VERSION 2.8)
project(randomwalk)

include_directories(/home/tim/install/naoqi-sdk-1.14-linux32/include)
link_directories(/home/tim/install/naoqi-sdk-1.14-linux32/lib)

find_package(qibuild)

qi_create_lib(randomwalk SHARED randomwalk.hpp randomwalk.cpp main.cpp)
qi_use_lib(randomwalk ALCOMMON BOOST)
