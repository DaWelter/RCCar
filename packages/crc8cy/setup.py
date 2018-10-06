#!/usr/bin/env python
from setuptools import setup
from Cython.Build import cythonize
import numpy

setup(
    name = "crc8cy",
    ext_modules = cythonize("crc8cy/_crc8cy.pyx"),
    include_dirs = [numpy.get_include(),],
    packages=['crc8cy'],
)
