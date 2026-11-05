from glob import glob

from setuptools import find_packages, setup

package_name = "inv_perception"

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
    description="Barcode, QR and AprilTag detection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "code_reader = inv_perception.code_reader:main",
            "tag_adapter = inv_perception.tag_adapter:main",
        ],
    },
)
