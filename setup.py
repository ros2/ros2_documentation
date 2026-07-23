#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os.path
from setuptools import setup

with open(os.path.join(os.path.dirname(__file__), "README.md")) as f:
    readme = f.read()

setup(
    name="sphinx-multiversion",
    description="Add support for multiple versions to sphinx",
    long_description=readme,
    long_description_content_type="text/markdown",
    classifiers=[
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
    ],
    author="Jan Holthuis",
    author_email="holthuis.jan@googlemail.com",
    url="https://holzhaus.github.io/sphinx-multiversion/",
    version="0.2.4",
    install_requires=["sphinx >= 2.1"],
    license="BSD",
    packages=["sphinx_multiversion"],
    entry_points={
        "console_scripts": ["sphinx-multiversion=sphinx_multiversion:main",],
    },
)
