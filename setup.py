# -*- coding: utf-8 -*-
# Function: Cython Compiler
#? Cython预编译执行器
#TODO Version 1.0.20230805
from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize(["Astar_cy.pyx","Identify_cy.pyx","MapScan_cy.pyx"],
    compiler_directives = { "language_level" : "3"}),
)