from glob import glob

from setuptools import find_packages, setup

package_name = "inv_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="CoreDrone team",
    maintainer_email="coredrone@example.com",
    description="EKF origin setter, parameter diff, bag recording.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "set_ekf_origin = inv_tools.set_ekf_origin:main",
            "fc_bootstrap = inv_tools.fc_bootstrap:main",
            "params_diff = inv_tools.params_diff:main",
            "record = inv_tools.record:main",
            "snapshot = inv_tools.snapshot:main",
        ],
    },
)
