from setuptools import find_packages, setup

package_name = "rehab_robot_xhand"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gu",
    maintainer_email="gu@localhost",
    description="ROS 2 adapter and safety layer for the vendor XHand SDK.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xhand_ros2_node = rehab_robot_xhand.xhand_ros2_node:main",
        ],
    },
)
