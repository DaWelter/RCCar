#!/usr/bin/env python

# Straight from https://en.wikibooks.org/wiki/Python_Programming/Extending_with_C

from distutils.core import setup
from distutils.extension import Extension

setup(name="h264decoder",
    ext_modules=[
        Extension(
          "h264decoder", 
          ["h264decoder.cpp", "h264decoder_python.cpp"],
          libraries = ["boost_python", "avcodec", "swscale", "avutil"],
          extra_compile_args = ["-O3", "--std=c++11"], )
    ])
