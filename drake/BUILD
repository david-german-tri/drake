# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

genrule(
    name = "doxygen_cxx",
    srcs = [
        # TODO: glob all the cc and h somehow
        "//drake/doc:doxyfile",
    ],
    outs = ["doxygen.tar.gz"],
    cmd = "doxygen $(location //drake/doc:doxyfile) && foobar",  #tar -czf $@ doxygen_cxx",
)
