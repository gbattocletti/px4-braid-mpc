from setuptools import find_packages, setup

package_name = "px4_braid_mpc"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gianpietro Battocletti",
    maintainer_email="g.battocletti@protonmail.com",
    description="Launch files for PX4-based simulation of braid MPC",
    license="GPL-3.0-or-later",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
