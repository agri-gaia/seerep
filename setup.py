#!/usr/bin/env python

"""
Two custom commands are added to the packaging process:
1. GeneratePythonFiles - Generates Python files from the Protobuf and Flatbuffer definitions. Additionally
   the import paths are changed to `seerep.pb` and `seerep.fbs`.
2. ChangeUtilPath - Changes the import path for the gRPC util scripts from `examples/python/gRPC/uitl` to `seerep/util`.

Note: The flatc and protoc compilers must be installed on the system running the packaging process. They are NOT
      installed automatically.
"""

import os
import shutil
from contextlib import suppress
from glob import glob
from pathlib import Path
from subprocess import call

from setuptools import Command, setup
from setuptools.command.build import build


class GeneratePythonFiles(Command):
    """A custom command to generate Python files from the .pb and .fbs definitions"""

    def initialize_options(self) -> None:
        self.proto_msgs_path = Path("seerep_msgs/protos/")
        self.proto_interface_path = Path("seerep_com/protos/")
        self.fbs_msgs_path = Path("seerep_msgs/fbs/")
        self.fbs_interface_path = Path("seerep_com/fbs/")
        self.bdist_dir = None

    def finalize_options(self) -> None:
        with suppress(Exception):
            self.bdist_dir = Path(self.get_finalized_command("bdist_wheel").bdist_dir)

    def from_pb(self) -> None:
        """
        The protoc compiler ignores the package directive for Python, since Python modules are organized according
        to their filesystem path. https://developers.google.com/protocol-buffers/docs/proto3#packages.
        The correct folder strcuture therefore has to be created manually. Additionally the import path inside the
        files need to be adjusted to the new structure with sed.
        """
        pb_dir = Path(self.bdist_dir / "seerep/pb/")
        pb_dir.mkdir(parents=True, exist_ok=True)
        Path(pb_dir / "__init__.py").touch()

        protoc_call = [
            "protoc",
            f"--proto_path={self.proto_msgs_path}",
            f"--proto_path={self.proto_interface_path}",
            f"--python_out={pb_dir}",
            f"--grpc_out={pb_dir}",
            f"--plugin=protoc-gen-grpc={shutil.which('grpc_python_plugin')}",
            *glob(f"{self.proto_msgs_path}/*.proto"),
            *glob(f"{self.proto_interface_path}/*.proto"),
        ]

        if call(protoc_call) != 0:
            raise Exception("protoc call failed")

        # Change the import paths in the files to new folder structure
        sed_call = [
            "sed",
            "-i",
            "-E",
            "s/^import.*_pb2/from seerep.pb &/",
            *glob(f"{pb_dir}/*.py"),
        ]

        if call(sed_call) != 0:
            raise Exception("sed call failed")

    def from_fb(self) -> None:
        """
        Currently the flatc compiler has a bug when using --python and --grpc, which prevents the use of -I and -o
        for specifying input and output directories https://github.com/google/flatbuffers/issues/7397.
        The current workaround is to copy the fbs files to the output directory, then run the flatc compiler in the
        output dir and remove the .fbs files afterwards.
        """
        shutil.copytree(self.fbs_msgs_path, self.bdist_dir, dirs_exist_ok=True)
        shutil.copytree(self.fbs_interface_path, self.bdist_dir, dirs_exist_ok=True)

        fbs_files = glob(f"{self.bdist_dir}/*.fbs")

        os.chdir(self.bdist_dir)

        flatc_call = [
            "flatc",
            "--python",
            "--grpc",
            # The compiler ONLY accepts the filenames, no paths!
            *[os.path.basename(file) for file in fbs_files],
        ]

        if call(flatc_call) != 0:
            raise Exception("flatc call not")

        os.chdir("../../..")

        # Remove the .fbs files
        for file in fbs_files:
            with suppress(Exception):
                os.remove(file)

    def run(self) -> None:
        self.from_pb()
        self.from_fb()


class ChangeUtilPath(Command):
    """
    Change the import path for the gRPC util scripts from 'examples/python/gRPC/uitl' to 'seerep/util' by copying
    into the bdist directory.
    """

    def initialize_options(self) -> None:
        self.current_util_path = Path("examples/python/gRPC/util/")
        self.new_util_path = Path("seerep/util/")
        self.bdist_dir = None

    def finalize_options(self) -> None:
        with suppress(Exception):
            self.bdist_dir = Path(self.get_finalized_command("bdist_wheel").bdist_dir)

    def run(self) -> None:
        new_path_in_bdist = Path(self.bdist_dir / self.new_util_path)
        shutil.copytree(self.current_util_path, new_path_in_bdist)
        Path(new_path_in_bdist / "__init__.py").touch()


class CustomBuild(build):
    sub_commands = [('build_python', None), ('change_util_path', None)] + build.sub_commands


setup(
    packages=[],
    cmdclass={'build': CustomBuild, 'build_python': GeneratePythonFiles, 'change_util_path': ChangeUtilPath},
)
