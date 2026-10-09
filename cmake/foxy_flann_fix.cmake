# Foxy FLANN and VTK compatibility fix
# This file ensures FLANN and VTK libraries are properly linked in ROS2 Foxy environment

# Find FLANN
find_package(FLANN QUIET)

if(NOT FLANN_FOUND)
    # Try to find FLANN manually
    find_path(FLANN_INCLUDE_DIR flann/flann.hpp
        PATHS /usr/include /usr/local/include
    )
    find_library(FLANN_LIBRARY
        NAMES flann_cpp flann
        PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
    )
    
    if(FLANN_INCLUDE_DIR AND FLANN_LIBRARY)
        # Create imported target
        if(NOT TARGET FLANN::FLANN)
            add_library(FLANN::FLANN UNKNOWN IMPORTED)
            set_target_properties(FLANN::FLANN PROPERTIES
                IMPORTED_LOCATION "${FLANN_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIR}"
            )
        endif()
        message(STATUS "Found FLANN (manual): ${FLANN_LIBRARY}")
    else()
        message(WARNING "FLANN library not found, creating dummy target")
        # Create a dummy interface library to avoid build errors
        if(NOT TARGET FLANN::FLANN)
            add_library(FLANN::FLANN INTERFACE IMPORTED)
        endif()
    endif()
else()
    # Create FLANN::FLANN target if it doesn't exist
    if(NOT TARGET FLANN::FLANN)
        if(FLANN_LIBRARIES)
            add_library(FLANN::FLANN UNKNOWN IMPORTED)
            set_target_properties(FLANN::FLANN PROPERTIES
                IMPORTED_LOCATION "${FLANN_LIBRARIES}"
                INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIRS}"
            )
        else()
            add_library(FLANN::FLANN INTERFACE IMPORTED)
        endif()
    endif()
    message(STATUS "Found FLANN: ${FLANN_INCLUDE_DIRS}")
endif()

# Fix VTK library names for Foxy (VTK 7.1 uses versioned library names)
# PCL brings in VTK dependencies but doesn't handle versioned names correctly
# Apply unconditionally since PCL dependency may be transitive
set(VTK_VERSION_SUFFIX "-7.1")

# List of common VTK libraries that need version suffix
set(VTK_LIBS_TO_FIX
    vtkChartsCore vtkCommonColor vtkCommonCore vtksys
    vtkCommonDataModel vtkCommonMath vtkCommonMisc vtkCommonSystem
    vtkCommonTransforms vtkCommonExecutionModel vtkFiltersGeneral
    vtkCommonComputationalGeometry vtkFiltersCore vtkInfovisCore
    vtkFiltersExtraction vtkFiltersStatistics vtkImagingFourier
    vtkImagingCore vtkalglib vtkRenderingContext2D vtkRenderingCore
    vtkFiltersGeometry vtkFiltersSources vtkRenderingFreeType
    vtkFiltersModeling vtkImagingSources vtkInteractionStyle
    vtkInteractionWidgets vtkFiltersHybrid vtkImagingColor
    vtkImagingGeneral vtkImagingHybrid vtkIOImage vtkDICOMParser
    vtkmetaio vtkRenderingAnnotation vtkRenderingVolume vtkIOXML
    vtkIOCore vtkIOXMLParser vtkIOGeometry vtkIOLegacy vtkIOPLY
    vtkRenderingLOD vtkViewsContext2D vtkViewsCore
    vtkRenderingContextOpenGL2 vtkRenderingOpenGL2
)

set(VTK_FIXED_COUNT 0)
foreach(vtk_lib ${VTK_LIBS_TO_FIX})
    # Only create if target doesn't exist
    if(NOT TARGET ${vtk_lib})
        # Find the versioned library
        find_library(${vtk_lib}_VERSIONED_LIBRARY
            NAMES ${vtk_lib}${VTK_VERSION_SUFFIX}
            PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
            NO_DEFAULT_PATH
        )
        
        if(${vtk_lib}_VERSIONED_LIBRARY)
            # Create an alias target
            add_library(${vtk_lib} UNKNOWN IMPORTED)
            set_target_properties(${vtk_lib} PROPERTIES
                IMPORTED_LOCATION "${${vtk_lib}_VERSIONED_LIBRARY}"
            )
            math(EXPR VTK_FIXED_COUNT "${VTK_FIXED_COUNT} + 1")
        endif()
    endif()
endforeach()

if(VTK_FIXED_COUNT GREATER 0)
    message(STATUS "Applied VTK 7.1 versioned library fix for Foxy: ${VTK_FIXED_COUNT} libraries")
endif()
