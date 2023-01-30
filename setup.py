#!/usr/bin/env python

"""
This script adds custom build steps to the python package build process.
The python classes for the protobuf and flatbuffers interface and messages
of SEEREP are generated using the respective compilers.

The protoc compiler is installed automatically via pip, while the flatc compiler
has to be installed manually (not available on pypi currently).
So if the build fails, make sure flatc is installed.
"""

import glob
import os
import shutil
import subprocess
from contextlib import suppress
from pathlib import Path
from typing import List

from setuptools import Command, setup
from setuptools.command.build import build


class CustomCommand(Command):
    def initialize_options(self) -> None:
        self.bdist_dir = None
        self.proto_msgs_path = None
        self.proto_api_path = None
        self.fbs_msgs_path = None
        self.fbs_api_path = None

    def finalize_options(self) -> None:
        self.proto_msgs_path = Path("seerep-msgs/protos/")
        self.proto_api_path = Path("seerep-com/protos/")
        self.fbs_msgs_path = Path("seerep-msgs/fbs/")
        self.fbs_api_path = Path("seerep-com/fbs/")
        with suppress(Exception):
            self.bdist_dir = Path(self.get_finalized_command("bdist_wheel").bdist_dir)

    @staticmethod
    def get_files(path: Path, file_ending: str) -> List[str]:
        if path.is_dir():
            return [str(file_path) for file_path in path.glob(f"*.{file_ending}")]
        else:
            return []

    def build_protos(self) -> None:
        if self.bdist_dir:
            """
            The protoc compiler ignores the package directive in the proto files.
            https://developers.google.com/protocol-buffers/docs/proto3#packages
            Therefore we need to manualy put it in the correct folder structure and post
            process it to comply with the existing flatbuffers structure.
            """
            output_dir = Path(self.bdist_dir / "seerep/pb/")
            output_dir.mkdir(parents=True, exist_ok=True)
            Path(self.bdist_dir / "seerep/__init__.py").touch()
            Path(self.bdist_dir / "seerep/pb/__init__.py").touch()

            protoc_call = [
                "python3",
                "-m",
                "grpc_tools.protoc",
                f"--proto_path={self.proto_msgs_path}",
                f"--proto_path={self.proto_api_path}",
                f"--python_out={output_dir}",
                f"--grpc_python_out={output_dir}",
                *CustomCommand.get_files(self.proto_msgs_path, "proto"),
                *CustomCommand.get_files(self.proto_api_path, "proto"),
            ]

            subprocess.call(protoc_call)

            sed_call = [
                "sed",
                "-i",
                "-E",
                "s/^import.*_pb2/from seerep.pb &/",
                *CustomCommand.get_files(output_dir, "py"),
            ]

            subprocess.call(sed_call)

    def build_fbs(self) -> None:
        if self.bdist_dir:
            output_dir = self.bdist_dir

            """
            Currently the flatc compiler has a bug when using --python and --grpc,
            which prevents the use of -I and -o for specifying input and output directories.
            https://github.com/google/flatbuffers/issues/7397

            The current workaround is to copy the fbs files to the output directory,
            then run the flatc compiler in the output dir and remove the fbs files afterwards.
            """

            shutil.copytree(self.fbs_msgs_path, self.bdist_dir, dirs_exist_ok=True)
            shutil.copytree(self.fbs_api_path, self.bdist_dir, dirs_exist_ok=True)

            os.chdir(output_dir)
            fbs_files = glob.glob("*.fbs")

            flatc_call = [
                "flatc",
                "--python",
                "--grpc",
                *fbs_files,
            ]

            subprocess.call(flatc_call)

            os.chdir("../../..")

            for file in fbs_files:
                with suppress(Exception):
                    os.remove(output_dir / file)

    def run(self) -> None:
        self.build_protos()
        self.build_fbs()


class CustomBuild(build):
    sub_commands = [('build_custom', None)] + build.sub_commands


setup(packages=[], cmdclass={'build': CustomBuild, 'build_custom': CustomCommand})
