from setuptools import find_packages, setup

package_name = "inv_layout"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="CoreDrone team",
    maintainer_email="coredrone@example.com",
    description="Warehouse layout model shared by mission, inventory and the sim world generator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "layout_info = inv_layout.cli:main",
        ],
    },
)
