from setuptools import setup

setup(
    name="deictic_registration",
    version="0.1.0",
    packages=["deictic_registration"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/deictic_registration"]),
        ("share/deictic_registration", ["package.xml", "LICENSE"]),
        ("share/deictic_registration/config", ["config/isaac_cpu.yaml", "config/isaac_balanced_cpu.yaml",
                                              "config/isaac_head1280_cuda.yaml",
                                              "config/isaac_head1280_cuda_05.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="ATR Lab",
    maintainer_email="atr@kent.edu",
    description="Calibrated SuperPoint/PnP registration for deictic frame fusion",
    license="Apache-2.0",
    entry_points={"console_scripts": ["registration_node = deictic_registration.node:main"]},
)
