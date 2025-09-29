# STEPFOC

# Spectral micro BLDC controller
[![License: GPLv3](https://img.shields.io/badge/license-GPLv3-blue)](https://opensource.org/license/gpl-3-0/) ![Issues](https://img.shields.io/github/issues/PCrnjak/STEPFOC-stepper-controller) ![release](https://img.shields.io/github/v/release/PCrnjak/STEPFOC-stepper-controller)

<img src="Photos/STEPFOC PCB.png" alt="drawing" width="5000"/>

**STEPFOC** is a FOC based stepper motor controller designed to deliver affordable, high-performance hardware paired with powerful open-source software. Its mission is to transform low-cost stepper motors into high-performance robotic servos, enabling advanced robotics applications without the high price tag.

The controller features a 14-bit magnetic encoder, inline current sensing, CAN and UART interfaces, and supports position, velocity, torque, and impedance control modes.

> [!NOTE]
> Optimized for NEMA17 format but it can also be attached to nema 23 and 14 steppers!


> [!IMPORTANT]
>     **STEPFOC stepper driver is built on the same hardware and software foundation as the Spectral Micro BLDC driver. As a result, some references or images in this documentation may still show the Spectral Micro, since both drivers share about 99% of their features. They also use the same CAN and UART command sets, with only minor differences in UART-based calibration and setup procedures.**

# Where to buy?

You can buy the Spectral Micro BLDC controller here: <br />
 https://source-robotics.com/products/spectral-micro-bldc-controller

# Firmware 
Spectral firmware is located [here](https://github.com/PCrnjak/Spectral-Micro-BLDC-controller/tree/main/Spectral%20BLDC%20Firmware). To install it follow [this guide!](https://source-robotics.github.io/Spectral-BLDC-docs/apage3_flashing_firmware/)


# Documentation:

- [Official website](https://source-robotics.com/products/spectral-micro-bldc-controller)
- [DOCS](https://source-robotics.github.io/Spectral-BLDC-docs/) Offers great starting guides with project examples + code 


# API and control
- [GUI software ](https://github.com/PCrnjak/Spectral-motor-GUI)
- [Python API & Examples](https://github.com/PCrnjak/Spectral-BLDC-Python/tree/main)
- [Source Robotics toolbox](https://github.com/PCrnjak/Source-Robotics-Toolbox/tree/main) 
- [SimpleFOC port](https://github.com/PCrnjak/Spectral-Micro-Simple-FOC-)
- **Early stage**: [Arduino](https://github.com/PCrnjak/SpectralMicroArduino), [ROS2](https://github.com/PCrnjak/SpectralMicroROS2)

# More about Spectral BLDC motor controllers

- [Source robotics website](https://www.youtube.com/channel/UCp3sDRwVkbm7b2M-2qwf5aQ)
- [Youtube](https://www.youtube.com/channel/UCp3sDRwVkbm7b2M-2qwf5aQ)
- [Hackaday](https://hackaday.io/project/191860-parol6-desktop-robotic-arm)
- [Instagram](https://www.instagram.com/source_robotics/)
- [DOCS](https://source-robotics.github.io/Spectral-BLDC-docs/)
- [Twitter](https://twitter.com/SourceRobotics)
- [Discord](https://discord.com/invite/prjUvjmGpZ )
- [Forum](https://discourse.source-robotics.com/)
- [Blog](https://source-robotics.com/blogs/blog)



# Liability 
1. The software and hardware are still in development and may contain bugs, errors, or incomplete features.
2. Users are encouraged to use this software and hardware responsibly and at their own risk.

# Support the project

The majority of this project is open source and freely available to everyone. Your assistance, whether through donations or advice, is highly valued. Thank you!

 [![General badge](https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white)](https://paypal.me/PCrnjak?locale.x=en_US)
[![General badge](https://img.shields.io/badge/Patreon-F96854?style=for-the-badge&logo=patreon&logoColor=white)](https://www.patreon.com/PCrnjak)

# Spectral firmware is under GPLv3 Licence
