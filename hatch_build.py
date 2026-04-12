import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

from hatchling.builders.hooks.plugin.interface import BuildHookInterface
from packaging import tags


class CustomHook(BuildHookInterface[Any]):
    """
    Custom build hook to compile the Meson project and package the resulting binaries.
    """

    source_dir = Path("build")
    target_dir = Path("vapoursynth/plugins")

    def initialize(self, version: str, build_data: dict[str, Any]) -> None:
        """
        Called before the build process starts.
        Sets build metadata and executes the Meson compilation.
        """
        # https://hatch.pypa.io/latest/plugins/builder/wheel/#build-data
        build_data["pure_python"] = False

        # Custom platform tagging logic:
        # We avoid the default 'infer_tag' (e.g., cp314-cp314-win_amd64) to prevent needing a separate wheel
        # for every Python version.
        # Since the compiled plugin only depends on the VapourSynth API and the OS/architecture,
        # we use a more generic tag: 'py3-none-<platform>'.
        #
        # NOTE:
        # For multi-platform distribution, this script should be run in a CI environment (like cibuildwheel)
        # or driven by environment variables to inject the appropriate platform tags.
        build_data["tag"] = f"py3-none-{next(tags.platform_tags())}"

        # Setup with vsenv
        # The ``--vsenv`` flag in the Meson setup command activates the Visual Studio environment on Windows,
        # which is required for MSVC-based compilation. On Linux and macOS, this flag is safely ignored.
        subprocess.run([sys.executable, "-m", "mesonbuild.mesonmain", "setup", "build", "--vsenv"], check=True)

        # Compile
        subprocess.run([sys.executable, "-m", "mesonbuild.mesonmain", "compile", "-C", "build"], check=True)

        # Ensure the target directory exists and copy the compiled binaries
        self.target_dir.mkdir(parents=True, exist_ok=True)
        for file_path in self.source_dir.glob("*"):
            if file_path.is_file() and file_path.suffix in [".dll", ".so", ".dylib"]:
                shutil.copy2(file_path, self.target_dir)

    def finalize(self, version: str, build_data: dict[str, Any], artifact_path: str) -> None:
        """
        Called after the build process finishes.
        Cleans up temporary build artifacts.
        """
        shutil.rmtree(self.target_dir.parent, ignore_errors=True)
