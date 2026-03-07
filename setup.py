from glob import glob
import os
from setuptools import find_packages, setup

package_name = "map_tools"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools", "websockets", "numpy"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="Tools for map-based no-go zones and Nav2 integration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zone_server = map_tools.web_zone_server:main",
            "web_zone_server = map_tools.web_zone_server:main",
        ],
    },
)
