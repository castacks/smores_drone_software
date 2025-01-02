# Smores Project

Welcome to the Smores project! This repository contains code for two main components: Fast-ACVNet and thermal_preprocessing. Each component has its own environment setup using `environment.yml` files.

## Table of Contents

- [Introduction](#introduction)
- [Setup](#setup)
    - [Fast-ACVNet Environment](#fast-acvnet-environment)
    - [Thermal Preprocessing Environment](#thermal-preprocessing-environment)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The Smores project provides the codebase for performing SLAM in dense smoke filled indoor environments with onboard compute. The project is divided into many components:

1. **Fast-ACVNet**: A fast and accurate stereo matching network.
2. **thermal_preprocessing**: A set of tools for preprocessing thermal images.

## Setup

To get started with the Smores project, you need to set up the environments for both components. Follow the instructions below to create and activate the environments.

### Fast-ACVNet Environment

1. Navigate to the `Fast-ACVNet` directory:
        ```
        cd src/Fast-ACVNet
        ```

2. Create the environment using the `environment.yml` file:
        ```
        conda env create -f environment.yml
        ```

3. Activate the environment:
        ```
        conda activate fast_acv
        ```

### Thermal Preprocessing Environment

1. Navigate to the `thermal_preprocessing` directory:
        ```
        cd src/thermal_preprocessing
        ```

2. Create the environment using the `environment.yml` file:
        ```
        conda env create -f environment.yml
        ```

3. Activate the environment:
        ```
        conda activate thermal_preprocessing
        ```

## Usage

Once the environments are set up and activated, you can start using the tools provided by each component. Refer to the respective documentation in each directory for detailed usage instructions.

- To run `Fast-ACV`, you can run the following command from the root directory:
 ```
 bash src/Fast-ACVNet/run_eval.sh
 ```
 - To run the `thermal_preprocessing` pipeline, you can run the following command from the root directory:
 ```
 python src/thermal-preprocessing/run_preprocessing.py
 ```

## Contributing

When developing you own modules please adhere to the following best practices:
- Include all the code under `src/` and the relevant data under `data/`. 
- Add directories that are not needed to `.gitignore`. Github will not host data greater than 100mb.
- Document your changes to this `readme.md` and other third party project management softwares.
- Add your changes, and always leave a comprehensive message with your commit using `git commit -m "<example-message>"`
- Push your code to a `<new or existing branch>` and not to `main`. 
- Assign a reviewer who can check and merge the code accordingly. We can follow 1 week merging cycles.
- If in doubt, ask instead of trial and error.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
