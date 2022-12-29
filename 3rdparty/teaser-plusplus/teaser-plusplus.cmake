include(ExternalProject)

ExternalProject_Add(
        ext_teaser
        PREFIX teaser
        GIT_REPOSITORY  https://gitlab.com/threeDVision/teaser-plusplus.git
        GIT_TAG 826f49e4adb98bae7de75c8092f2c0a64d2c2a78
        UPDATE_COMMAND ""
        CMAKE_ARGS
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DBUILD_TESTS=OFF
        -DBUILD_TEASER_FPFH=OFF
        -DBUILD_PYTHON_BINDINGS=OFF
        -DBUILD_DOC=OFF
        CONFIGURE_HANDLED_BY_BUILD ON
        ${ExternalProject_CMAKE_ARGS_hidden}
        BUILD_BYPRODUCTS
        <INSTALL_DIR>/${Open3D_INSTALL_LIB_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}teaser_registration${CMAKE_STATIC_LIBRARY_SUFFIX}
)

ExternalProject_Get_Property(ext_teaser INSTALL_DIR)
set(TEASER_INCLUDE_DIRS ${INSTALL_DIR}/include/) # "/" is critical.
set(TEASER_LIB_DIR ${INSTALL_DIR}/lib)
set(TEASER_LIBRARIES teaser_registration pmc)
