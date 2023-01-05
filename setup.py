#!/usr/bin/env python

from pathlib import Path

from dunamai import Version
from setuptools import setup

this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name='seerep-grpc',
    version=Version.from_any_vcs().serialize(metadata=False),
    description='package for the SEEREP gRPC API',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Mark Niemeyer',
    author_email='mark.niemeyer@dfki.de',
    url='https://github.com/agri-gaia/seerep/',
    packages=['seerep-msgs-fb', 'seerep-msgs-pb', 'seerep-com-fb', 'seerep-com-pb'],
    license_files=('LICENSE',),
)
