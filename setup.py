from importlib import metadata

import setuptools
import setuptools_scm

scm_version = metadata.version("setuptools_scm")

version_file = "python/lsst/ts/m2/version.py"
if scm_version.startswith("8"):
    setuptools.setup(
        version=setuptools_scm.get_version(
            version_file=version_file,
            relative_to="pyproject.toml",
        )
    )
else:
    setuptools.setup(version=setuptools_scm.get_version(write_to=version_file))
