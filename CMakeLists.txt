cmake_minimum_required(VERSION 3.13)
project(CRSF-Projects LANGUAGES CXX)

# === configure ====================================================================================
set(CRSF_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/crsf" CACHE PATH "Directory Path for CRSF SDK.")

if(NOT EXISTS ${CRSF_SDK_DIR})
    message(FATAL_ERROR "'crsf' directory does NOT exist!\nYou can download from http://crseed.org")
endif()

# uncomment this to add examples
#add_subdirectory("crsf/examples")

# list projects
add_subdirectory("CRHands")
# ==================================================================================================
