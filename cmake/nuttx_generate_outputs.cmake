# ##############################################################################
# cmake/nuttx_generate_outputs.cmake
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

function(nuttx_generate_outputs target config)
  if(CONFIG_INTELHEX_BINARY)
    add_custom_command(
      OUTPUT ${config}.hex
      COMMAND ${CMAKE_OBJCOPY} -O ihex ${target} ${config}.hex
      DEPENDS ${target})
    add_custom_target(${config}-hex ALL DEPENDS ${config}.hex)
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "${config}.hex\n")
  endif()

  if(CONFIG_MOTOROLA_SREC)
    add_custom_command(
      OUTPUT ${config}.srec
      COMMAND ${CMAKE_OBJCOPY} -O srec ${target} ${config}.srec
      DEPENDS ${target})
    add_custom_target(${config}-srec ALL DEPENDS ${config}.srec)
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "${config}.srec\n")
  endif()

  if(CONFIG_RAW_BINARY)
    add_custom_command(
      OUTPUT ${config}.bin
      COMMAND ${CMAKE_OBJCOPY} -O binary ${target} ${config}.bin
      DEPENDS ${target})
    add_custom_target(${config}-bin ALL DEPENDS ${config}.bin)
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "${config}.bin\n")
  endif()

  if(CONFIG_ELF_32_L)
  add_custom_command(
    OUTPUT ${config}.elf
    COMMAND ${CMAKE_OBJCOPY} -O elf32-littlearm ${target} ${config}.elf
    DEPENDS ${target})
  add_custom_target(${config}-elf ALL DEPENDS ${config}.elf)
  file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "${config}.elf\n")
  endif()

endfunction(nuttx_generate_outputs)
