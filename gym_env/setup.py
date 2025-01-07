from setuptools import setup, find_packages

setup(
    name="custom_gym_env",  # Name of your package
    version="0.1",  # Version of your package
    description="Custom Gymnasium environment for cube lift task",
    packages=find_packages(),  # Automatically discover all packages in your project
    install_requires=[
        "gymnasium",  # Make sure Gymnasium is installed
        "numpy",  # Basic dependencies, add any others that you directly use
        "torch",  # Assuming your environment uses PyTorch
    ],
    python_requires="=3.10",  # Adjust Python version if necessary
)
