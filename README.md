# Camera-LiDAR calibration

This tool was developed to perform **intrinsic** and **extrinsic** calibration between camera and LiDAR sensors in a simple and intuitive manner. This was designed to make the process less tedious, even for those with little experience in the field.

---

## Table of Contents
1. [Explanation of intrinsic and extrinsic matrices](#explanation-of-intrinsic-and-extrinsic-matrices)
2. [Get Started](#get-started)
   - [Prerequisites](#prerequisites)
   - [Installation](#installation)
3. [Usage](#usage)
   - [Node Overview](#node-overview)
   - [Workflow](#workflow)
   - [Running Nodes](#running-nodes)


---

## Explanation of intrinsic and extrinsic matrices

### Intrinsic calibration (<a href="https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html">For more information click here</a>)

#### Distortion coefficients 

Some cameras introduce significant distortion into images. 
The two main types of distortion are:
- <b>Radial distortion</b> makes straight lines appear curved, and it increases as points move away from the center of the image. It can be represented like this:
   - $x_{distorted} = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$
   - $y_{distorted} = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$
- <b>Tangential distortion</b> occurs because the lens capturing the image is not aligned perfectly parallel to the shooting plane. Therefore, some areas of the image may appear closer than expected. It can be represented like this:
   - $x_{distorted} = x + [2p_1xy + p_2(r^2 + 2x^2)]$
   - $y_{distorted} = y + [p_1(r^2 + 2y^2) + 2p_2xy]$

In short, you need to find five parameters, known as distortion coefficients, given by:
- $Distortion \text{ } coefficients = (k_1,  k_2, p_1, p_2, k_3)$

#### Camera matrix

In addition to this, additional information is needed: the camera's <b>intrinsic parameters</b>. Intrinsic parameters are specific to a camera.
The <b>focal length</b> and <b>optical centers</b> can be used to create a camera matrix, which can be used to remove distortion due to a specific camera's lenses.
- $camera \text{ } matrix =\begin{bmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}$

### Extrinsic calibration (<a href="https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html?utm_source=chatgpt.com">For more information click here</a>)

Extrinsic parameters corresponds to rotation and translation vectors which translates a coordinates of a 3D point to a 2D coordinate system.

#### Rotation matrix
rvec is a vector in axis-angle format:
- The <b>direction</b> of the vector indicates the rotation axis.
- The <b>length</b> (magnitude) of the vector represents the rotation angle in radians around that axis.
   - $rvec = \begin{bmatrix}
   r_x\\
   r_y\\
   r_z
   \end{bmatrix}$
- In other words: 
   - The <b>magnitude</b> of the vector represents the rotation angle $\theta$ in radians.  
   $\theta = ||rvec|| = \sqrt{r_x^2 + r_y^2 + r_z^2}$
   - The <b>direction</b> of the vector indicates the axis of rotation $k$ (unit).  
   $r = \frac{rvec}{\theta} = \left[\frac{r_x}{\theta}, \frac{r_y}{\theta}, \frac{r_z}{\theta}\right]$

Using Rodrigues' formula:
- $R = cos(\theta)I + (1-cos\theta)rr^T + sin(\theta)
\begin{bmatrix}
0 & -r_z & r_y\\
r_z & 0 & -r_x\\
-r_y & r_x & 0
\end{bmatrix}$,  
   where R is a 3x3 matrix.


#### Translation vector
The translation vector $t$ 3×1 moves the origin of the world coordinates to the origin (optical center) of the camera.
- $t = \begin{bmatrix}t_x \\ t_y \\ t_z\end{bmatrix}$

#### Projection of points on the camera
To project the lidar points onto the camera, the following formula is used:
- $P_c = RP_w + t$  
or in homogeneous coordinates:
- $P_c = \begin{bmatrix}
R & t\\
0 & 1 \end{bmatrix} P_w$  
      - where: $P_c = \begin{bmatrix}X_c \\ Y_c \\ Z_c \\ 1\end{bmatrix}$ and $P_w = \begin{bmatrix}X_w \\ Y_w \\ Z_w \\ 1\end{bmatrix}$

---

## Get Started

### Prerequisites

To run this package, ensure the following dependencies are installed:
- **Git**: to download the repository and to download new updates.
- **Python 3.10**: Python version with which the tool was tested
- **Ubuntu 22.04**

### Installation

#### Clone the Repository
Start by cloning the repository:
```bash
git clone https://github.com/ddp22/Calibration_tool.git
```

#### Python virtual environment
Before you start, create the python virtual environment and install all the necessary modules:
```bash
cd Calibration_tool
python3 -m venv calibration_tool_venv
source ./calibration_tool_venv/bin/activate
python3 -m pip install -r requirements.txt
```

---


## Usage

### Calibration Tool Overview
This tool is divided into two parts: first the intrinsic calibration is performed and then the extrinsic one.

| **Calibration type**           | **Description**                                                                                       | **Output**                                     |
|--------------------------|-------------------------------------------------------------------------------------------------------|-----------------------------------------------|
| `intrinsic calibration`  | Calculate the two intrinsic matrices `camera_matrix` and `dist_coeffs` from a rosbag                                                   | `camera_matrix` and `dist_coeffs`       .            |
| `extrinsic calibration`    | Calculate the two extrinsic matrices `tvec` and `rvec` from a rosbag.                                             | `tvec`, `rvec`, `R` and `extrinsic_matrix`.

### Preliminary steps

Before starting the calibration, collect a rosbag consisting of the camera and lidar topics.  
In the case of <b>Mivia car</b> the two topics are `/camera/compressed` and `/lidar/projected_camera`.  
The rosbag must contain a video in which a chessboard must be shown to the camera, in a clearly visible manner, in different positions (rotating the board a little) and at different distances, so that all the area of interest is covered.

### General configuration

After creating the Rosbag, add the information to the `./config/general_configuration.yaml` file:
```yaml
lidar:
  lidar_topic: /velodyne_points # lidar topic

camera:
  image_topic: /camera/compressed # camera topic (this topic is supposed to publish compressed images)
  image_size:
    width: 1440 # image width
    height: 1080 # height image

chessboard:
  pattern_size: # number of internal vertices of the board (rows and columns are interchangeable)
    rows: 16
    columns: 18
  square_size_meters: 0.43 # size of the chessboard squares in meters
```


### Intrinsic calibration

[Click here to see the intrinsic calibration](./intrisic_calibration.md).

### Extrinsic calibration

[Click here to see the extrinsic calibration](./extrinsic_calibration.md).

