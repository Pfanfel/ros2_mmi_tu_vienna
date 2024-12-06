from setuptools import find_packages, setup

package_name = "color_led"

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
    maintainer="michi",
    maintainer_email="michael.smirnov@gmx.de",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arduino_publisher = color_led.arduino_publisher:main",
            "arduino_subscriber = color_led.arduino_subscriber:main",
            "keyboard_publisher = color_led.keyboard_publisher:main",
        ],
    },
)
