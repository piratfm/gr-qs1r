# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR} ${LIBUSB_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS} ${LIBUSB_LIBRARY_DIRS})
list(APPEND qs1r_sources
    qs1r_src_impl.cc qs_io_libusb.cc )

add_library(gnuradio-qs1r SHARED ${qs1r_sources})
target_link_libraries(gnuradio-qs1r ${Boost_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} ${LIBUSB_LIBRARIES})
set_target_properties(gnuradio-qs1r PROPERTIES DEFINE_SYMBOL "gnuradio_qs1r_EXPORTS")

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-qs1r
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_qs1r_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_qs1r.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_qs1r.cc
)

add_executable(test-qs1r ${test_qs1r_sources})

target_link_libraries(
  test-qs1r
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  gnuradio-qs1r
)

GR_ADD_TEST(test_qs1r test-qs1r)
