from setuptools import find_packages, setup

package_name = "rehab_robot_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="gu",
    maintainer_email="gu@localhost",
    description="Read-only validation and monitoring tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rehab_config_validator = rehab_robot_control.config_validator:main",
            "rehab_runtime_probe = rehab_robot_control.runtime_probe:main",
            "rehab_topic_monitor = rehab_robot_control.topic_monitor:main",
            "xhand_control_ros2_node = rehab_robot_control.xhand_ros2_node:main",
            "rehab_empty_grasp_dry_run = rehab_robot_control.empty_grasp_dry_run:main",
        ],
    },
)
