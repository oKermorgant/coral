# Custom compilation of osgocean
# No installation, only lib

cmake_minimum_required (VERSION 3.5 FATAL_ERROR)

project (osgOcean)

SET(OSGOCEAN_SRC ${PROJECT_SOURCE_DIR}/osgocean)
# remove warnings from osgOcean / not my code
add_compile_options(-w)
set(CMAKE_BUILD_TYPE Release)

# Work around for building an xcode project. It seems XCode projects that
# built with CMake are not compatible with relative paths.
IF(NOT APPLE)
  SET( CMAKE_USE_RELATIVE_PATHS true )
ENDIF()

find_package (osg REQUIRED)
include_directories (${OSG_INCLUDE_DIR})

find_package (osgDB REQUIRED)
include_directories (${OSGDB_INCLUDE_DIR})

find_package (osgGA REQUIRED)
include_directories (${OSGGA_INCLUDE_DIR})

find_package (osgViewer REQUIRED)
include_directories (${OSGVIEWER_INCLUDE_DIR})

find_package (osgUtil REQUIRED)
include_directories (${OSGUTIL_INCLUDE_DIR})

find_package (osgText REQUIRED)
include_directories (${OSGTEXT_INCLUDE_DIR})

find_package (OpenThreads REQUIRED)
include_directories(${OPENTHREADS_INCLUDE_DIR})


IF (NOT WIN32)
  IF (NOT OSG_LIBRARY_DEBUG)
    SET(OSG_LIBRARY_DEBUG ${OSG_LIBRARY})
  ENDIF()
  IF (NOT OSGDB_LIBRARY_DEBUG)
    SET(OSGDB_LIBRARY_DEBUG ${OSGDB_LIBRARY})
  ENDIF()
  IF (NOT OSGGA_LIBRARY_DEBUG)
    SET(OSGGA_LIBRARY_DEBUG ${OSGGA_LIBRARY})
  ENDIF()
  IF (NOT OSGVIEWER_LIBRARY_DEBUG)
    SET(OSGVIEWER_LIBRARY_DEBUG ${OSGVIEWER_LIBRARY})
  ENDIF()
  IF (NOT OSGUTIL_LIBRARY_DEBUG)
    SET(OSGUTIL_LIBRARY_DEBUG ${OSGUTIL_LIBRARY})
  ENDIF()
  IF (NOT OSGTEXT_LIBRARY_DEBUG)
    SET(OSGTEXT_LIBRARY_DEBUG ${OSGTEXT_LIBRARY})
  ENDIF()
  IF (NOT OPENTHREADS_LIBRARY_DEBUG)
    SET(OPENTHREADS_LIBRARY_DEBUG ${OPENTHREADS_LIBRARY})
  ENDIF()
ENDIF()

set(OpenGL_GL_PREFERENCE GLVND)
find_package (OpenGL REQUIRED)
include_directories (${OPENGL_INCLUDE_DIR})

OPTION(USE_FFTW3 "Use FFTW3 (double-precision) (GPL) as FFT library." OFF) # Use FFTW3 by default.
OPTION(USE_FFTW3F "Use FFTW3 (single-precision) (GPL) as FFT library." ON) 
OPTION(USE_FFTSS "Use FFTSS (LGPL) as FFT library." OFF)                       

set(CMAKE_MODULE_PATH "${OSGOCEAN_SRC}/CMakeModules;${CMAKE_MODULE_PATH}")

# How do I enforce that only one of USE_FFTW3, USE_FFTW3F or USE_FFTSS should
# be selected at one time? Is there the concept of a radio-button or
# single-selection list in CMake? Right now it will just use the first one
# that is checked in the order below.

IF(USE_FFTSS)
  find_package (fftss REQUIRED)
  MESSAGE(STATUS "Using FFTSS (LGPL) as FFT library.")

  ADD_DEFINITIONS(-DUSE_FFTSS)
  SET( FFT_INCLUDE_DIR ${FFTSS_INCLUDE_DIR} )
  SET( FFT_LIBRARY ${FFTSS_LIBRARY} )

  SET(USE_FFTW3 FALSE)
  SET(USE_FFTW3F FALSE)
ELSEIF(USE_FFTW3F)
  find_package (fftw3f-3 REQUIRED)
  MESSAGE(STATUS "Using FFTW3F (single-precision) (GPL) as FFT library.")

  ADD_DEFINITIONS(-DUSE_FFTW3F)
  SET( FFT_INCLUDE_DIR ${FFTW3F-3_INCLUDE_DIR} )
  SET( FFT_LIBRARY ${FFTW3F-3_LIBRARY} )

  SET(USE_FFTW3 FALSE)
  SET(USE_FFTSS FALSE)
ELSEIF(USE_FFTW3)
  find_package (fftw3-3 REQUIRED)
  MESSAGE(STATUS "Using FFTW3 (double-precision) (GPL) as FFT library.")

  ADD_DEFINITIONS(-DUSE_FFTW3)
  SET( FFT_INCLUDE_DIR ${FFTW3-3_INCLUDE_DIR} )
  SET( FFT_LIBRARY ${FFTW3-3_LIBRARY} )

  SET(USE_FFTW3F FALSE)
  SET(USE_FFTSS FALSE)
ELSE()
  # Error if none of the three is selected.
  MESSAGE("No FFT library selected, you will not be able to generate ocean surfaces.")
ENDIF()

SET(HEADER_PATH ${OSGOCEAN_SRC}/include/osgOcean )

SET( SHADER_FILES
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_ocean_surface.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_ocean_surface.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_ocean_surface_vbo.vert

  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_godrays.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_godrays.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_godray_screen_blend.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_godray_screen_blend.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_godray_glare.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_godray_glare.frag

  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_silt_quads.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_silt_quads.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_silt_points.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_silt_points.frag

  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_streak.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_streak.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_glare_composite.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_glare_composite.frag

  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_downsample_glare.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_downsample.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_downsample.frag

  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_gaussian.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_gaussian1.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_gaussian2.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_dof_combiner.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_dof_combiner.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_water_distortion.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_water_distortion.frag

  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_ocean_scene.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_ocean_scene.frag
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_ocean_scene_lispsm.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_ocean_scene_lispsm.frag
  
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_heightmap.vert
  ${OSGOCEAN_SRC}/resources/shaders/osgOcean_heightmap.frag
)


SET( LIB_HEADERS
  ${HEADER_PATH}/Cylinder
  ${HEADER_PATH}/DistortionSurface
  ${HEADER_PATH}/FFTOceanTechnique
  ${HEADER_PATH}/FFTOceanSurface
  ${HEADER_PATH}/FFTOceanSurfaceVBO
  ${HEADER_PATH}/FFTSimulation
  ${HEADER_PATH}/GodRays
  ${HEADER_PATH}/GodRayBlendSurface
  ${HEADER_PATH}/MipmapGeometry
  ${HEADER_PATH}/MipmapGeometryVBO
  #${HEADER_PATH}/OceanScene
  ${HEADER_PATH}/OceanTechnique
  ${HEADER_PATH}/OceanTile
  ${HEADER_PATH}/RandUtils
  ${HEADER_PATH}/ScreenAlignedQuad
  ${HEADER_PATH}/ShaderManager
  ${HEADER_PATH}/SiltEffect
  ${HEADER_PATH}/WaterTrochoids
  ${HEADER_PATH}/Export
  ${HEADER_PATH}/Version
)

ADD_DEFINITIONS(-DOSGOCEAN_LIBRARY)

INCLUDE_DIRECTORIES (
   ${OPENSCENEGRAPH_INCLUDE_DIR}
   ${OPENGL_INCLUDE_DIR}
   ${OSG_INCLUDE_DIR}
   ${FFT_INCLUDE_DIR}
   ${OSGOCEAN_SRC}/include/
)

set(OSGOCEAN_CPP ${OSGOCEAN_SRC}/src/osgOcean)

ADD_LIBRARY(
  osgOcean
  SHARED
  ${LIB_HEADERS}
  ${OSGOCEAN_CPP}/Cylinder.cpp
  ${OSGOCEAN_CPP}/DistortionSurface.cpp
  ${OSGOCEAN_CPP}/FFTOceanTechnique.cpp
  ${OSGOCEAN_CPP}/FFTOceanSurface.cpp
  ${OSGOCEAN_CPP}/FFTOceanSurfaceVBO.cpp
  ${OSGOCEAN_CPP}/FFTSimulation.cpp
  ${OSGOCEAN_CPP}/GodRays.cpp
  ${OSGOCEAN_CPP}/GodRayBlendSurface.cpp
  ${OSGOCEAN_CPP}/MipmapGeometry.cpp
  ${OSGOCEAN_CPP}/MipmapGeometryVBO.cpp
  #${OSGOCEAN_CPP}/OceanScene.cpp
  ${OSGOCEAN_CPP}/OceanTechnique.cpp
  ${OSGOCEAN_CPP}/OceanTile.cpp
  ${OSGOCEAN_CPP}/ScreenAlignedQuad.cpp
  ${OSGOCEAN_CPP}/ShaderManager.cpp
  ${OSGOCEAN_CPP}/SiltEffect.cpp
  ${OSGOCEAN_CPP}/WaterTrochoids.cpp
  ${OSGOCEAN_CPP}/Version.cpp
  ${SHADER_FILES}
)

SET_TARGET_PROPERTIES(
  osgOcean
  PROPERTIES
  PUBLIC_HEADER "${LIB_HEADERS}"
)

TARGET_LINK_LIBRARIES(
  osgOcean PUBLIC
  optimized ${OSG_LIBRARY}
  debug ${OSG_LIBRARY_DEBUG}
  optimized ${OSGUTIL_LIBRARY}
  debug ${OSGUTIL_LIBRARY_DEBUG}
  optimized ${OSGDB_LIBRARY}
  debug ${OSGDB_LIBRARY_DEBUG}
  optimized ${OSGGA_LIBRARY}
  debug ${OSGGA_LIBRARY_DEBUG}
  optimized ${OPENTHREADS_LIBRARY}
  debug ${OPENTHREADS_LIBRARY_DEBUG}
  ${OPENGL_LIBRARY}
  ${FFT_LIBRARY}
)

