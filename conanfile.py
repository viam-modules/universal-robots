import os
import re
import tarfile

from tempfile import TemporaryDirectory

from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, load

class universal_robots(ConanFile):
    name = "viam-universal-robots"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/universal-robots"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    def export_sources(self):
        for pat in ["CMakeLists.txt", "LICENSE", "src/*", "meta.json*", "*.sh"]:
            copy(self, pat, self.recipe_folder, self.export_sources_folder)

    def set_version(self):
        self.version = "0.3.1"

    def requirements(self):
        # NOTE: If you update the `viam-cpp-sdk` dependency here, it
        # should also be updated in `bin/setup.{sh,ps1}`.
        self.requires("viam-cpp-sdk/0.19.0")
        self.requires("eigen/[>=3.3]")
        self.requires("boost/[>=1.74.0]")

    def validate(self):
        check_min_cppstd(self, 17)

    def validate(self):
        check_min_cppstd(self, 17)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def layout(self):
        cmake_layout(self, src_folder=".")

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def deploy(self):
        with TemporaryDirectory(dir=self.deploy_folder) as tmp_dir:
            self.output.debug(f"Creating temporary directory {tmp_dir}")
            self.output.info("Copying bin and lib")
            for dir in ["bin", "lib"]:
                copy(self, "*", src=os.path.join(self.package_folder, dir), dst=os.path.join(tmp_dir, dir))

            self.output.info("Copying scripts and additional files")
            for pat in ["*.sh", "meta.json"]:
                copy(self, pat, src=self.package_folder, dst=tmp_dir)

            self.output.info("Creating module.tar.gz")
            with tarfile.open("module.tar.gz", "w|gz") as tar:
                tar.add(tmp_dir, ".")

                self.output.debug("module.tar.gz contents:")
                for mem in tar.getmembers():
                    self.output.debug(mem.name)
