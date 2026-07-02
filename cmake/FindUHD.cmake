find_path(UHD_INCLUDE_DIR
    NAMES uhd/usrp/multi_usrp.hpp
    HINTS "${GNSS_UHD_ROOT}/include" "$ENV{UHD_DIR}/include"
)

find_library(UHD_LIBRARY
    NAMES uhd
    HINTS "${GNSS_UHD_ROOT}/lib" "$ENV{UHD_DIR}/lib"
)

find_path(UHD_BOOST_INCLUDE_DIR
    NAMES boost/config.hpp
    HINTS
        "${GNSS_UHD_ROOT}/include"
        "$ENV{BOOST_ROOT}"
        "$ENV{CONDA_PREFIX}/Library/include"
        "$ENV{CONDA_PREFIX}/include"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(UHD
    REQUIRED_VARS UHD_INCLUDE_DIR UHD_LIBRARY UHD_BOOST_INCLUDE_DIR
)

if(UHD_FOUND AND NOT TARGET UHD::uhd)
    add_library(UHD::uhd UNKNOWN IMPORTED)
    set_target_properties(UHD::uhd PROPERTIES
        IMPORTED_LOCATION "${UHD_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${UHD_INCLUDE_DIR};${UHD_BOOST_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(UHD_INCLUDE_DIR UHD_LIBRARY UHD_BOOST_INCLUDE_DIR)
