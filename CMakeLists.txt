# Author: Jan Brejcha <brejcha@adobe.com>, <brejcha@gmail.com>, <ibrejcha@fit.vutbr.cz>
# Project Immersive Trip Reports 2017-2018
# AdobePatentID="P7840-US"
#
# Project: LandscapeAR 2018
#
# Copyright 2020 CPhoto@FIT, Brno University of Technology,
# Faculty of Information Technology,
# Božetěchova 2, 612 00, Brno, Czech Republic
#
# Redistribution and use in source code form, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# 3. Redistributions must be pursued only for non-commercial research
#    collaboration and demonstration purposes.
#
# 4. Where separate files retain their original licence terms
#    (e.g. MPL 2.0, Apache licence), these licence terms are announced, prevail
#    these terms and must be complied.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.

project(itr)

MACRO(register_definitions DEF)
  add_definitions(${DEF})
  string(REPLACE "-D" "" STRIPPED_DEF ${DEF})
  list(APPEND OpenMVG_DEFINITIONS ${STRIPPED_DEF})
ENDMACRO()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message("clang detected: using std::unordered_map for Hash_Map")
  register_definitions(-DOPENMVG_STD_UNORDERED_MAP)
endif()

set(CMAKE_MACOSX_RPATH 1)

set(ITR_VERSION_MAJOR 0)
set(ITR_VERSION_MINOR 9)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

configure_file (
  "${PROJECT_SOURCE_DIR}/config/ITRConfig.h.in"
  "${PROJECT_BINARY_DIR}/ITRConfig.h"
  )

add_subdirectory(src)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty)
