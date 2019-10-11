cmake_minimum_required(VERSION 2.8.3)
project(camera)

#SET(CMAKE_MODULE_PATH
#  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmakeFindModules)

find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
#find_package(Glew REQUIRED)
find_package(JPEG REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  yidamsg
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
# SiftGPU：手动设置其头文件与库文件所在位置
include_directories(${OpenGL_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${JPEG_INCLUDE_DIR})
set(CORE_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCCore.so)
set(hpr_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libhpr.so)
set(hcnetsdk_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libhcnetsdk.so)
set(HCPreview_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCPreview.so)
set(analyzedata_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libanalyzedata.so)
set(HCAlarm_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCAlarm.so)
set(HCCoreDevCfg_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCCoreDevCfg.so)
set(HCDisplay_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCDisplay.so)
set(HCGeneralCfgMgr_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCGeneralCfgMgr.so)
set(HCIndustry_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCIndustry.so)
set(HCPlayBack_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCPlayBack.so)
set(HCVoiceTalk_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libHCVoiceTalk.so)
set(iconv2_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libiconv2.so)
set(StreamIransClient_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libStreamTransClient.so)
set(SystemTransform_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libSystemTransform.so)
set(PlayCtrl_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libPlayCtrl.so)
set(AudioRender_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libAudioRender.so)
set(SuperRender_LIBRARY ${PROJECT_SOURCE_DIR}/LIBS/libSuperRender.so)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_executable( capture src/main.cpp )
target_link_libraries( capture
    ${OpenCV_LIBS}
    ${GLEW_LIBRARY} 
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${CORE_LIBRARY} 
    ${hpr_LIBRARY}
    ${hcnetsdk_LIBRARY}
    ${catkin_LIBRARIES}
    ${HCPreview_LIBRARY}
    ${analyzedata_LIBRARY}
    ${HCAlarm_LIBRARY}
    ${HCCoreDevCfg_LIBRARY} 
    ${HCDisplay_LIBRARY}
    ${HCGeneralCfgMgr_LIBRARY}
    ${HCIndustry_LIBRARY} 
    ${HCPlayBack_LIBRARY}
    ${HCVoiceTalk_LIBRARY}
    ${iconv2_LIBRARY}
    ${StreamIransClient_LIBRARY}
    ${SystemTransform_LIBRARY}
    ${PlayCtrl_LIBRARY}
    ${AudioRender_LIBRARY}
    ${SuperRender_LIBRARY}
)


