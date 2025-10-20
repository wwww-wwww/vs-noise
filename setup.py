import platform

from setuptools import Extension, setup
from setuptools.command.build_clib import build_clib
from setuptools.command.build_ext import build_ext

sources = [
    "AddNoise/AddNoise.cpp",
    "AddNoise/perlin.cpp",
    "AddNoise/simplex.cpp",
]
libraries = []

machine = platform.machine()
if machine == "i386" or machine == "x86_64":
    sources += [
        "AddNoise/VCL2/instrset_detect.cpp",
        "AddNoise/AddNoise_SSE2.cpp",
    ]

    libraries += [
        [
            "vsnoise_avx2",
            {
                "sources": ["AddNoise/AddNoise_AVX2.cpp"],
                "include_dirs": ["AddNoise", "/usr/local/include/vapoursynth"],
            },
        ],
        [
            "vsnoise_avx512",
            {
                "sources": ["AddNoise/AddNoise_AVX512.cpp"],
                "include_dirs": ["AddNoise", "/usr/local/include/vapoursynth"],
            },
        ],
    ]


class build_ext_compiler_check(build_ext):
    def run(self):
        super().run()
        for ext in self.extensions:
            if ext.name == "vsnoise":
                print("adding", self.build_lib)
                ext.library_dirs.append(self.build_lib)

    def build_extension(self, ext):
        compiler_type = self.compiler.compiler_type
        if compiler_type in ["unix", "mingw32"]:
            ext.extra_compile_args += ["-march=native"]

        super().build_extension(ext)


class build_clib_compiler_check(build_clib):
    def build_libraries(self, libraries) -> None:
        gcc = self.compiler.compiler_type in ["unix", "mingw32"]
        for library in libraries:
            if library[0] == "vsnoise_avx2":
                if gcc:
                    library[1]["cflags"] = ["-mfma", "-mavx2"]
                else:
                    library[1]["cflags"] = ["/arch:AVX2"]
            if library[0] == "vsnoise_avx512":
                if gcc:
                    library[1]["cflags"] = [
                        "-mfma",
                        "-mavx512f",
                        "-mavx512bw",
                        "-mavx512dq",
                        "-mavx512vl",
                    ]
                else:
                    library[1]["cflags"] = ["/arch:AVX512"]

        super().build_libraries(libraries)


setup(
    name="vsnoise",
    version=1,
    packages=["vsnoise"],
    ext_modules=[
        Extension(
            name="vsnoise_ext",
            sources=sources,
            include_dirs=["AddNoise", "/usr/local/include/vapoursynth"],
            libraries=["vapoursynth", "vsnoise_avx2", "vsnoise_avx512"],
        ),
    ],
    cmdclass={
        "build_ext": build_ext_compiler_check,
        "build_clib": build_clib_compiler_check,
    },
    libraries=libraries,
)
