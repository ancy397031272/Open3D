include(ExternalProject)



ExternalProject_Add(
        ext_tinyspline
        PREFIX tinyspline
        URL https://github.com/msteinbeck/tinyspline/archive/refs/tags/v0.6.0.tar.gz
        URL_HASH SHA256=3ea31b610dd279266f26fd7ad5b5fca7a20c0bbe05c7c32430ed6aa54d57097a
        DOWNLOAD_DIR "${OPEN3D_THIRD_PARTY_DOWNLOAD_DIR}/tinyspline"
        UPDATE_COMMAND ""
        CMAKE_ARGS
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DTINYSPLINE_INSTALL_LIBRARY_DIR=<INSTALL_DIR>/lib
        -DBUILD_TESTING=OFF
        -DTINYSPLINE_BUILD_DOCS=OFF
        -DTINYSPLINE_BUILD_EXAMPLES=OFF
        -DTINYSPLINE_BUILD_TESTS=OFF
        -DTINYSPLINE_ENABLE_CXX=ON
        -DTINYSPLINE_ENABLE_PYTHON=OFF
        -DTINYSPLINE_WARNINGS_AS_ERRORS=OFF
        CONFIGURE_HANDLED_BY_BUILD ON
        ${ExternalProject_CMAKE_ARGS_hidden}
        BUILD_BYPRODUCTS
        <INSTALL_DIR>/${Open3D_INSTALL_LIB_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}tinyspline${CMAKE_STATIC_LIBRARY_SUFFIX}
)

ExternalProject_Get_Property(ext_tinyspline INSTALL_DIR)
set(TINYSPLINE_INCLUDE_DIRS ${INSTALL_DIR}/include/) # "/" is critical.
set(TINYSPLINE_LIB_DIR ${INSTALL_DIR}/${Open3D_INSTALL_LIB_DIR})
set(TINYSPLINE_LIBRARIES tinyspline)

