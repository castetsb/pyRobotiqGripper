<p align="center">
  <a href="https://pyrobotiqgripper.readthedocs.io/en/latest/">
    <img src="https://img.shields.io/badge/Full%20Documentation-00B0F0?style=for-the-badge&logo=readthedocs&logoColor=white" alt="Full Documentation">
  </a>
  <br>
  <sub>👉 <a href="https://pyrobotiqgripper.readthedocs.io/en/latest/">pyrobotiqgripper.readthedocs.io</a> -- installation, usage guides and the full API reference live there, this README is just a quick overview.</sub>
</p>

<!-- --8<-- [start:intro] -->
# pyRobotiqGripper
<p style="margin-top: -15px; color: var(--md-typeset-color); font-size: 1.1em; font-style: italic;">Python Driver for Robotiq Grippers.</p>

<p align="center">
  <a href="https://pepy.tech/projects/pyrobotiqgripper"><img src="https://static.pepy.tech/personalized-badge/pyrobotiqgripper?period=total&units=INTERNATIONAL_SYSTEM&left_color=BLACK&right_color=GREEN&left_text=downloads" alt="PyPI Downloads"></a>
  <img src="https://img.shields.io/pypi/dm/pyrobotiqgripper" alt="PyPI Monthly Downloads">
  <img src="https://img.shields.io/github/stars/castetsb/pyrobotiqgripper" alt="GitHub Stars">
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/castetsb/pyRobotiqGripper/master/docs/_static/logo.png" alt="pyRobotiqGripper logo">
</p>

pyRobotiqGripper is a Python library designed to facilitate control of Robotiq grippers using Modbus RTU communication via serial port or over ethernet.
It is compatible with 2F85, 2F140, and Hande.
<!-- --8<-- [end:intro] -->

<!-- GitHub strips <iframe>, so a clickable thumbnail is used here instead;
     the docs site embeds the video inline (see docs/index.md). -->
<p align="center">
  <a href="https://www.youtube.com/watch?v=82S5LgefvJo">
    <img src="https://raw.githubusercontent.com/castetsb/pyRobotiqGripper/master/docs/_static/intro_video.jpg" alt="pyRobotiqGripper introduction video">
  </a>
</p>

<!-- --8<-- [start:disclaimer] -->

## Disclaimer

This library can be seen as a starting point for a Robotiq integration project.
You are responsible for what you do with this library.
The author takes no responsibility for any malfunction.
<!-- --8<-- [end:disclaimer] -->

> [!NOTE]
> This python package is in constant evolution. Some breaking change may happen from one version to another. This can mainly impact the high level functions.

> [!TIP]
> **Other related software tool**
> C++, ROS2 and other software tools are available on the Robotiq GitHub page: [Robotiq GitHub Repository](https://github.com/robotiq).
