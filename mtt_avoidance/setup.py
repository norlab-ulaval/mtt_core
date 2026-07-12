from glob import glob
from setuptools import find_packages, setup


package_name = "mtt_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MTT",
    maintainer_email="mtt@example.com",
    description="Shadow-first Nav2 avoidance and Teach route rejoin orchestration for MTT.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mtt_avoidance_supervisor = mtt_avoidance.supervisor_node:main",
            "mtt_rejoin_planner = mtt_avoidance.rejoin_planner_node:main",
            "mtt_composite_path_validator = mtt_avoidance.composite_path_validator_node:main",
            "mtt_avoidance_executor = mtt_avoidance.executor_node:main",
            "mtt_recovery_manager = mtt_avoidance.recovery_manager_node:main",
            "mtt_avoidance_command_mux = mtt_avoidance.command_mux_node:main",
        ],
    },
)
