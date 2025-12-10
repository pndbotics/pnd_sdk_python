# Overview

<div align="center">

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![CycloneDDS](https://img.shields.io/badge/CycloneDDS-0.10.2-6D28D9)](https://cyclonedds.io/)

![Updated At](https://img.shields.io/badge/Updated_At-December-64748B?style=flat-square)
![Version](https://img.shields.io/badge/Version-1.0.3-2563EB?style=flat-square)
![Supported Robot](https://img.shields.io/badge/Supported-Adam_Lite/Standard/Pro-4f46e5?style=flat-square)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-059669?style=flat-square)](https://opensource.org/licenses/BSD-3-Clause)
[![Issues](https://img.shields.io/badge/issues-open-EF4444?style=flat-square)](https://github.com/pndbotics/pnd_sdk_python/issues)

**Lightweight Python bindings for PND SDK, supporting robot state acquisition and low‑level control via DDS communication.**

</div>

## ✨ Features
- Pythonic interface for **PND LowCmd / LowState**
- Supports **request–response** and **topic publish/subscribe**
- Works both with **real robots** and **simulators**

## 📋 Table of Contents
- [Installation](#-installation)
- [Usage Examples](#-usage-examples)
- [FAQ](#-faq)
- [Contributing](#-contributing)
- [License](#-license)
- [Contact](#-contact)
- [Version Log](#-version-log)

## 🛠 Installation

### Dependencies
- Python ≥ 3.8  
- cyclonedds == 0.10.2  
- numpy  
- opencv-python  

### Install from Source
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/pndbotics/pnd_sdk_python.git
cd pnd_sdk_python
pip3 install -e .
```

## 📖 Usage Examples

The Python sdk interface maintains consistency with the pnd_sdk interface, achieving robot status acquisition and control through request-response or topic subscription/publishing.

Example programs are located in:  
```
/example
```

Ensure robot networking is configured correctly:  
https://wiki.pndbotics.com

### Low-Level Example
```bash
cd ~/example/adam_u/low_level
python3 adam_u_low_level_example.py
```

## ❓ FAQ

### 1. Error during installation:
```
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```

Compile and install **cyclonedds**:

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

Then install **pnd_sdk_python**:
```bash
cd ~/pnd_sdk_python
export CYCLONEDDS_HOME="~/cyclonedds/install"
pip3 install -e .
```

More info: https://pypi.org/project/cyclonedds/#installing-with-pre-built-binaries

## 🤝 Contributing

Contributions are welcome.

Feel free to open issues or pull requests.

## 📄 License

[BSD-3 Clause © PNDbotics](./LICENSE)

## 📖 Reference

- [pnd_mujoco](https://github.com/pndbotics/pnd_mujoco)


## 📞 Contact

- Email: info@pndbotics.com
- Wiki: https://wiki.pndbotics.com  
- SDK: https://github.com/pndbotics/pnd_sdk_python  
- Issues: https://github.com/pndbotics/pnd_mujoco/issues

## 📜 Version Log

| Version | Date       | Updates                                                                              |
| ------- | ---------- | ------------------------------------------------------------------------------------ |
| v1.0.3  | 2025-12-10 | - Add sp and lite </br> - Change wireless remote </br> - Add ddq and motor state|
| v1.0.2  | 2025-11-20 | Add pd and add hand in example|
| v1.0.1  | 2025-11-11 | Add hand support|
| v1.0.0  | 2025-11-07 | Initial release|

---

<div align="center">

[![Website](https://img.shields.io/badge/Website-PNDbotics-black?)](https://www.pndbotics.com)
[![Twitter](https://img.shields.io/badge/Twitter-@PNDbotics-1DA1F2?logo=twitter&logoColor=white)](https://x.com/PNDbotics)
[![YouTube](https://img.shields.io/badge/YouTube-ff0000?style=flat&logo=youtube&logoColor=white)](https://www.youtube.com/@PNDbotics)
[![Bilibili](https://img.shields.io/badge/-bilibili-ff69b4?style=flat&labelColor=ff69b4&logo=bilibili&logoColor=white)](https://space.bilibili.com/303744535)

**⭐ Star us on GitHub — it helps!**

</div>
