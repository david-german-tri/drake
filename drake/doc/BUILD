# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

genrule(
    name = "doxyfile",
    srcs = ["Doxyfile_CXX.in"],
    outs = ["Doxyfile_CXX"],
    cmd = "cat $< " +
          "| sed s/@drake_SOURCE_DIR@/drake/g " +
          "| sed s/@CMAKE_CURRENT_BINARY_DIR@/./g " +
          "| sed s/@DOXYGEN_DOT_FOUND@/True/g " +
          "> $@",
    visibility = ["//visibility:public"],
)
