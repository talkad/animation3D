# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt" AND EXISTS "C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitinfo.txt" AND
  "C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt" IS_NEWER_THAN "C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "C:/Users/pijon/OneDrive/Desktop/animation3D/cmake/../external/imgui"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'C:/Users/pijon/OneDrive/Desktop/animation3D/cmake/../external/imgui'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "D:/Program Files (x86)/Git/cmd/git.exe" -c http.sslVerify=false
            clone --no-checkout --config "advice.detachedHead=false" --config "advice.detachedHead=false" "https://github.com/ocornut/imgui.git" "imgui"
    WORKING_DIRECTORY "C:/Users/pijon/OneDrive/Desktop/animation3D/cmake/../external"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/ocornut/imgui.git'")
endif()

execute_process(
  COMMAND "D:/Program Files (x86)/Git/cmd/git.exe" -c http.sslVerify=false
          checkout "v1.69" --
  WORKING_DIRECTORY "C:/Users/pijon/OneDrive/Desktop/animation3D/cmake/../external/imgui"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'v1.69'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "D:/Program Files (x86)/Git/cmd/git.exe" -c http.sslVerify=false
            submodule update --recursive --init 
    WORKING_DIRECTORY "C:/Users/pijon/OneDrive/Desktop/animation3D/cmake/../external/imgui"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'C:/Users/pijon/OneDrive/Desktop/animation3D/cmake/../external/imgui'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitinfo.txt" "C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'C:/Users/pijon/OneDrive/Desktop/animation3D/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt'")
endif()
