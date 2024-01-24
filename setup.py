import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dog_robot"

NODE_LIST = {"movement": ["motor_client_async"], "sound": ["random_sound"]}


def name_helper(module: str, node_name: str):
    return f"{node_name} = {package_name}.{module}.{node_name}:main"


def entry_points_helper():
    entry_points = []
    for module, nodes in NODE_LIST.items():
        for node in nodes:
            entry_points.append(name_helper(module, node))
    return entry_points


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "": glob(os.path.join(package_name, "sound", "*.wav")),
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name),
            glob(os.path.join("launch", "*launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    include_package_data=True,
    zip_safe=True,
    maintainer="zaidan",
    maintainer_email="zaidanyahya28@gmail.com",
    description="A dog robot built using raspberry pi and raspimouse.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": entry_points_helper(),
    },
)
