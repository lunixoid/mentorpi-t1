from setuptools import setup

package_name = "mentorpi_calibration"

setup(
    name=package_name,
    version="0.4.0",
    packages=[package_name, package_name + ".stages"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Roman Kalashnikov",
    maintainer_email="lunix0x@gmail.com",
    description="Sensor mount calibration file and algorithms for MentorPi T1 (SD012).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calib = mentorpi_calibration.cli:main",
        ],
    },
)
