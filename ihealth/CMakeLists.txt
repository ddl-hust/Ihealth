cmake_minimum_required(VERSION 3.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# set the filepath to find third libaray
set(OpenCV_DIR D:/OpenCV3_4/x86)
set(OpenCV_INCLUDE_DIR  D:/OpenCV3_4/opencv-3.4.1/include)

# set(Boost_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Boost_1_67_0)
set(Boost_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Boost_1_67_0)
set(Boost_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Boost_1_67_0/source/boost_1_67_0)

set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen/eigen3_3_4 )

set(TBB_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/TBB)
# set(TBB_INCLUDE_DIR CMAKE_CURRENT_SOURCE_DIR/TBB/include)
set(TBB_LIBRARY  ${CMAKE_CURRENT_SOURCE_DIR}/TBB/lib/ia32/vc14/tbb.lib)

set(NI_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/NI)
# set(NI_INCLUDE_DIR CMAKE_CURRENT_SOURCE_DIR/NI/include)
set(NI_LIBRARY  ${CMAKE_CURRENT_SOURCE_DIR}/NI/lib/NIDAQmx.lib)

set(PUPILTRACKER_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pupiltracker)
set(PUPILTRACKER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pupiltracker/include)
set(PUPILTRACKER_LIBRARY  ${CMAKE_CURRENT_SOURCE_DIR}/pupiltracker/lib/pupiltracker.lib)

set(APS_ROOT_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/APS)
set(APS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/APS/include)
set(APS_LIBRARY   ${CMAKE_CURRENT_SOURCE_DIR}/APS/lib/APS168.lib)

set(MYSQL_ROOT_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/mysql)
# set(MYSQL_INCLUDE_DIR CMAKE_CURRENT_SOURCE_DIR/mysql/include)
set(MYSQL_LIBRARY   ${CMAKE_CURRENT_SOURCE_DIR}/mysql/lib/libmysql.lib)

set(MSC_ROOT_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/msc)
# set(MSC_INCLUDE_DIR CMAKE_CURRENT_SOURCE_DIR/msc)
set(MSC_LIBRARY    ${CMAKE_CURRENT_SOURCE_DIR}/msc/msc.lib)

set( WKE_ROOT_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/wke)
set(WKE_INCLUDE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/wke)
set(WKE_LIBRARY    ${CMAKE_CURRENT_SOURCE_DIR}/wke/wke.lib)

set( ZPLAY_ROOT_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/zplay)
set(ZPLAY_INCLUDE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/zplay/include)
set(ZPLAY_LIBRARY    ${CMAKE_CURRENT_SOURCE_DIR}/zplay/lib/libzplay.lib)

set(SPDLOG_INCLUDE_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include) #这样就不要每次手动在include
# # list(APPEND CMAKE_MODULE_PATH "${SPDLOG_DIR}/cmake")
# find_package(SPDLOG REQUIRED)

find_package(OpenCV REQUIRED)
find_package (Boost REQUIRED)
find_package(EIGEN)
find_package(TBB)
find_package(NI)
find_package(MYSQL)
find_package(MSC)
find_package(WKE)
find_package(ZPLAY)

include_directories(
${CMAKE_CURRENT_SOURCE_DIR} 
${CMAKE_CURRENT_SOURCE_DIR}/ATI/ATIDAQ 
${PUPILTRACKER_INCLUDE_DIR} 
${APS_INCLUDE_DIR} 
${MYSQL_INCLUDE_DIR} 
${APS_INCLUDE_DIR} 
${MSC_INCLUDE_DIR} 
${WKE_INCLUDE_DIR} 
${OpenCV_INCLUDE_DIR} 
${Boost_INCLUDE_DIR}
${EIGEN_INCLUDE_DIR} 
${NI_INCLUDE_DIR}
${ZPLAY_INCLUDE_DIR}
${TBB_INCLUDE_DIR}
${SPDLOG_INCLUDE_DIR}
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} Root_src)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ATI/ATIDAQ ATI_src)

file(GLOB Root_header ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB ATI_header ${CMAKE_CURRENT_SOURCE_DIR}/ATI/ATIDAQ/*.h ${CMAKE_CURRENT_SOURCE_DIR}/ATI/ATIDAQ/*.hpp)

set(ATI_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ATI/ATIDAQ)
list(REMOVE_ITEM ATI_src "${ATI_SRC_DIR}/events.c" "${ATI_SRC_DIR}/xmltok_impl.c" "${ATI_SRC_DIR}/xmltok_ns.c")
list(REMOVE_ITEM ATI_header "${ATI_SRC_DIR}/ascii.h" "${ATI_SRC_DIR}/asciitab.h" "${ATI_SRC_DIR}/ftsharedrt.h" "${ATI_SRC_DIR}/iasciitab.h" "${ATI_SRC_DIR}/latin1tab.h" 
"${ATI_SRC_DIR}/nametab.h" "${ATI_SRC_DIR}/utf8tab.h" "${ATI_SRC_DIR}/xmldef.h" "${ATI_SRC_DIR}/xmltok_impl.h")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /sdl-")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /sdl-")

ADD_DEFINITIONS(-D_AFXDLL)
add_executable(ihealth ${Root_src} ${ATI_src} ${Root_header} ${ATI_header})
add_dependencies(ihealth duilib CommonExt)

source_group("Source Files" FILES ${Root_src})
source_group("Source Files\\ATI" FILES ${ATI_src})
source_group("Header Files" FILES ${Root_header})
source_group("Header Files\\ATI" FILES ${ATI_header})

target_link_libraries(ihealth duilib CommonExt ${PUPILTRACKER_LIBRARY} ${APS_LIBRARY} ${MYSQL_LIBRARY} ${MSC_LIBRARY} ${WKE_LIBRARY} ${NI_LIBRARY} ${OpenCV_LIBRARIES} 
${TBB_LIBRARY} 
${ZPLAY_LIBRARY} 
)
if(WIN32)
   set_target_properties(ihealth PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:windows")
   set_target_properties(ihealth PROPERTIES COMPILE_DEFINITIONS_DEBUG "/SUBSYSTEM:windows")
   set_target_properties(ihealth PROPERTIES LINK_FLAGS_RELWITHDEBINFO "/SUBSYSTEM:windows")
   set_target_properties(ihealth PROPERTIES COMPILE_DEFINITIONS_RELWITHDEBINFO "/SUBSYSTEM:windows")
   set_target_properties(ihealth PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:windows")
   set_target_properties(ihealth PROPERTIES LINK_FLAGS_MINSIZEREL "/SUBSYSTEM:windows")
endif(WIN32)
