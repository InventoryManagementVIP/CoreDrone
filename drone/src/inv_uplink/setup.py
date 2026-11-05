from glob import glob

from setuptools import find_packages, setup

package_name = "inv_uplink"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="CoreDrone team",
    maintainer_email="coredrone@example.com",
    description="Store-and-forward uplink and CSV export.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["uplink = inv_uplink.node:main"]},
)
