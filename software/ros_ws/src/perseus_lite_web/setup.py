from setuptools import find_packages, setup
import os
from glob import glob

package_name = "perseus_lite_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "static"),
            glob("perseus_lite_web/static/*"),
        ),
    ],
    include_package_data=True,
    package_data={package_name: ["static/*"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nigel_H-S",
    maintainer_email="1388693+DingoOz@users.noreply.github.com",
    description="Lightweight telemetry dashboard for Perseus Lite (HTTP + SSE)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_node = perseus_lite_web.web_node:main",
        ],
    },
)
