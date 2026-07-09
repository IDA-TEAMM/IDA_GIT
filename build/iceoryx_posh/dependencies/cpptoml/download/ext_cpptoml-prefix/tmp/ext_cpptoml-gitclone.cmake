
if(NOT "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp/ext_cpptoml-gitinfo.txt" IS_NEWER_THAN "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp/ext_cpptoml-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp/ext_cpptoml-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/skystrife/cpptoml.git" "src"
    WORKING_DIRECTORY "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/skystrife/cpptoml.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout v0.1.1 --
  WORKING_DIRECTORY "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'v0.1.1'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp/ext_cpptoml-gitinfo.txt"
    "/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp/ext_cpptoml-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/root/ros2_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp/ext_cpptoml-gitclone-lastrun.txt'")
endif()

