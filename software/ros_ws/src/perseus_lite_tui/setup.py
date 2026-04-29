from setuptools import find_packages, setup

package_name = "perseus_lite_tui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nigel_H-S",
    maintainer_email="1388693+DingoOz@users.noreply.github.com",
    description="Curses TUI for tuning Perseus Lite roam (frontier_explorer) parameters at runtime.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perseus_lite_tui = perseus_lite_tui.perseus_lite_tui:main",
        ],
    },
)
