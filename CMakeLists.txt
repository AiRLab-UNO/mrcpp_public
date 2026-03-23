cmake_minimum_required(VERSION 3.16)
project(mrcpp)

# Set policy for download timestamps
#cmake_policy(SET CMP0135 NEW)

set(CMAKE_CXX_STANDARD 17)
include(ExternalProject)
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories(include)

if(WIN32)
    link_directories(lib/windows)
else()
    link_directories(lib/linux)
endif()

# ============================================================
# LKH-3 static library
# ============================================================
# On Windows the pre-compiled lib/liblkh.a is a Linux ELF binary.
# Build LKH from source instead so it produces a Windows PE/COFF archive.
if(WIN32)
    add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/mTSP/LKH-3.0.13/SRC
        ${CMAKE_CURRENT_BINARY_DIR}/lkh_build
    )
    set(LKH_LINK_TARGET lkh)           # CMake target from the sub-directory
else()
    # On Linux use the pre-compiled archive
    set(LKH_LINK_TARGET ${PROJECT_SOURCE_DIR}/lib/linux/liblkh.a)
endif()

# ============================================================
# External Build: BehaviorTree.CPP
# ============================================================
set(BTCPP_INSTALL_DIR ${CMAKE_BINARY_DIR}/install)

# Platform-specific library paths
if(WIN32)
    # MinGW debug builds append 'd' to the library name (e.g. libbehaviortree_cppd.dll)
    # Release builds use no suffix (libbehaviortree_cpp.dll)
    set(BTCPP_LIB_PATH_DEBUG   ${BTCPP_INSTALL_DIR}/bin/libbehaviortree_cppd.dll)
    set(BTCPP_IMP_PATH_DEBUG   ${BTCPP_INSTALL_DIR}/lib/libbehaviortree_cppd.dll.a)
    set(BTCPP_LIB_PATH_RELEASE ${BTCPP_INSTALL_DIR}/bin/libbehaviortree_cpp.dll)
    set(BTCPP_IMP_PATH_RELEASE ${BTCPP_INSTALL_DIR}/lib/libbehaviortree_cpp.dll.a)
    set(BTCPP_BYPRODUCTS
        ${BTCPP_LIB_PATH_DEBUG}   ${BTCPP_IMP_PATH_DEBUG}
        ${BTCPP_LIB_PATH_RELEASE} ${BTCPP_IMP_PATH_RELEASE})

    ExternalProject_Add(BehaviorTreeCPP
        URL https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/4.7.0.zip
        DOWNLOAD_NAME BehaviorTreeCPP-4.7.0.zip
        DOWNLOAD_DIR ${CMAKE_BINARY_DIR}/downloads
#        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR .
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${BTCPP_INSTALL_DIR}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_FLAGS=-Wa,-mbig-obj
        -DBUILD_SHARED_LIBS=ON
        -DBTCPP_EXAMPLES=OFF
        -DBTCPP_BUILD_TOOLS=OFF
        -DBUILD_TESTING=OFF
        -DBTCPP_SQLITE_LOGGING=OFF
        -DBTCPP_GROOT2_PUBLISHER=OFF
        -DBTCPP_GROOT_INTERFACE=OFF
#        -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        INSTALL_DIR ${BTCPP_INSTALL_DIR}
        UPDATE_DISCONNECTED TRUE
        BUILD_BYPRODUCTS ${BTCPP_BYPRODUCTS}
    )
else()
    set(BTCPP_LIB_PATH_RELEASE ${BTCPP_INSTALL_DIR}/lib/libbehaviortree_cpp.so)
    set(BTCPP_LIB_PATH_DEBUG   ${BTCPP_LIB_PATH_RELEASE})
    set(BTCPP_BYPRODUCTS       ${BTCPP_LIB_PATH_RELEASE})
    ExternalProject_Add(BehaviorTreeCPP
        URL https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/4.7.0.zip
        DOWNLOAD_NAME BehaviorTreeCPP-4.7.0.zip
        DOWNLOAD_DIR ${CMAKE_BINARY_DIR}/downloads
#        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR .
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${BTCPP_INSTALL_DIR}
        -DBUILD_SHARED_LIBS=ON
        -DBTCPP_EXAMPLES=OFF
        -DBTCPP_BUILD_TOOLS=OFF
        -DBUILD_TESTING=OFF
        -DBTCPP_SQLITE_LOGGING=OFF
#        -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        INSTALL_DIR ${BTCPP_INSTALL_DIR}
        UPDATE_DISCONNECTED TRUE
        BUILD_BYPRODUCTS ${BTCPP_LIB_PATH}
)
endif()



# IMPORTED SHARED target — no source files for IMPORTED libraries
add_library(BehaviorTreeCPP_lib SHARED IMPORTED GLOBAL)

if(WIN32)
    # Set per-configuration IMPORTED_LOCATION / IMPORTED_IMPLIB so both
    # Debug and Release build types resolve to the correct file.
    set_target_properties(BehaviorTreeCPP_lib PROPERTIES
            IMPORTED_LOCATION_DEBUG   ${BTCPP_LIB_PATH_DEBUG}
            IMPORTED_IMPLIB_DEBUG     ${BTCPP_IMP_PATH_DEBUG}
            IMPORTED_LOCATION_RELEASE ${BTCPP_LIB_PATH_RELEASE}
            IMPORTED_IMPLIB_RELEASE   ${BTCPP_IMP_PATH_RELEASE}
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
            MAP_IMPORTED_CONFIG_MINSIZEREL     Release
    )
else()
    set_target_properties(BehaviorTreeCPP_lib PROPERTIES
            IMPORTED_LOCATION ${BTCPP_LIB_PATH_RELEASE}
    )
endif()

add_dependencies(BehaviorTreeCPP_lib BehaviorTreeCPP)

# ============================================================
# ASIO / Boost
# ============================================================
# Crow uses CROW_USE_BOOST to pick up Boost.Asio instead of standalone asio
if(WIN32)
    set(BOOST_ROOT "C:/libs/boost_1_90_0/boost_1_90_0" CACHE PATH "Boost root" FORCE)
    include_directories(${BOOST_ROOT})
    # CROW_USE_BOOST   : use Boost.Asio instead of standalone asio
    # WIN32_LEAN_AND_MEAN / NOUSER : keep Windows headers minimal;
    #   NOUSER prevents winuser.h from being loaded, avoiding the
    #   IsChild(HWND,HWND) function declaration that conflicts with
    #   LKH's "extern int IsChild" global variable.
    add_compile_definitions(CROW_USE_BOOST WIN32_LEAN_AND_MEAN NOUSER)
else()
    #sudo apt install libasio-dev
endif()
include_directories(Crow/include)

# ============================================================
# Source files
# ============================================================
set(MRCPP_SOURCE_FILES
        src/mrcpp/MinRect.cpp
        src/mrcpp/SwathGenerator.cpp
        src/mrcpp/RayCasting.cpp
        src/mtsp/mTSP.cpp
        src/mrcpp/ObstacleAvoidance.cpp
)

# ============================================================
# Add Executable Macro
# ============================================================
macro(add_mrcpp_demo demo_name)
    add_executable(${ARGV} ${MRCPP_SOURCE_FILES})
    add_dependencies(${demo_name} BehaviorTreeCPP_lib)

    target_include_directories(${demo_name} PRIVATE
            include
            ${BTCPP_INSTALL_DIR}/include
            # LKH headers — point to the source INCLUDE dir
            ${CMAKE_CURRENT_SOURCE_DIR}/mTSP/LKH-3.0.13/SRC/INCLUDE
    )

    # On Windows/MinGW: big-obj needed for debug builds; link ws2_32 + mswsock for Boost.Asio
    if(WIN32)
        target_compile_options(${demo_name} PRIVATE -Wa,-mbig-obj)
        target_link_libraries(${demo_name}
                PRIVATE
                ${LKH_LINK_TARGET}
                BehaviorTreeCPP_lib
                ws2_32
                mswsock
        )
    else()
        target_link_libraries(${demo_name}
                PRIVATE
#               Python3::Python
                ${LKH_LINK_TARGET}
                BehaviorTreeCPP_lib
        )
    endif()
endmacro(add_mrcpp_demo)

# ============================================================
# Build Executables
# ============================================================
add_mrcpp_demo(mrcpp_basic interface/mrcpp_basic.cpp src/metric/EnergyCalculator.cpp src/metric/utils.cpp)
add_mrcpp_demo(mrcpp_ablation interface/mrcpp_ablation.cpp src/metric/EnergyCalculator.cpp src/metric/utils.cpp)
add_mrcpp_demo(energy_estimator interface/energy_estimator.cpp src/metric/EnergyCalculator.cpp src/metric/utils.cpp)

# ============================================================
# CPack configuration
# ============================================================
set(CPACK_GENERATOR "DEB")

# These two are REQUIRED by Debian format
set(CPACK_PACKAGE_CONTACT "redwan@example.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Redwan Newaz")

# Optional but recommended
set(CPACK_PACKAGE_NAME "mrcpp")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "mrcpp server and pyviz tools for multirobot coverage")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.27), libpython3.8 (>= 3.8)")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.27)")
include(CPack)
