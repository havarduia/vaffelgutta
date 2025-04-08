# Vaffelgutta

## Overview

This repository contains the code and documentation for our bachelor's project at the University of Agder. The project focuses on integrating and controlling the **Interbotix VX300s** robotic arm and the **Intel RealSense D455** camera using **ROS 2 Humble**.

---

## Hardware Components

- **Interbotix VX300s Robotic Arm**  
  A versatile robotic arm suitable for various manipulation tasks.  
  [Interbotix Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/index.html)

- **Intel RealSense D455 Camera**  
  A depth camera providing high-accuracy 3D scanning capabilities.  
  [RealSense GitHub](https://github.com/IntelRealSense/realsense-ros)

---

## Software Requirements

- **Operating System**: Ubuntu 22.04  
- All other requirements are installed in setup.

---

## Installation

1. **Clone This Repository**

   ```bash
   git clone https://github.com/havarduia/vaffelgutta.git
   cd vaffelgutta
2. **Install Python Dependencies**
   
   ```bash
   pip install -r requirements.txt
4. **Run setup_1**
   
   ```bash
   cd setup
   ./setup_1.sh
6. **Run setup_2**
   
   ```bash
   ./setup_2.sh
8. **Run setup_3**
   
   ```bash
   ./setup_3.sh
10. **Done!**
    
   You can now run main.py!
