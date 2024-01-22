from setuptools import find_packages, setup

package_name = "dog_robot"

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
    maintainer="zaidan",
    maintainer_email="zaidanyahya28@gmail.com",
    description="A dog robot built using raspberry pi and raspimouse.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
