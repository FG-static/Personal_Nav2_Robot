# Copyright 2020 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# copied from osqp_vendor/osqp_vendor-extras.cmake

list(APPEND osqp_vendor_TARGETS osqp)

# OSQP 0.6 exports INTERFACE_INCLUDE_DIRECTORIES as <prefix>/include/osqp,
# but this workspace includes "osqp/osqp.h". Also expose <prefix>/include.
if(TARGET osqp::osqp)
  get_target_property(_osqp_inc osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
  if(_osqp_inc)
    foreach(_inc IN LISTS _osqp_inc)
      get_filename_component(_inc_parent "${_inc}" DIRECTORY)
      if(EXISTS "${_inc_parent}/osqp/osqp.h")
        set_property(TARGET osqp::osqp APPEND PROPERTY
          INTERFACE_INCLUDE_DIRECTORIES "${_inc_parent}")
      endif()
    endforeach()
  endif()
endif()
