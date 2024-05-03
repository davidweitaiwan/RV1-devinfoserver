*`Established: 2024/05/03`* *`Updated: 2024/05/03`*

## About The Project
The vehicle device information management service for the robot vehicle ver. 1 project.

This service is used to manage the information of the devices connected to the vehicle. The service will store the information with DevInfo structure (`vehicle_interfaces/msg_content/DevInfo.msg`), and dump the information to JSON file by each node if new device information is registered.

The service will resolve the conflict of the DevInfo structure by comparing the node name, hostname, IP address, and MAC address. If the conflict is detected, the service will update the information with the latest information.

**NOTE:** The implementation of communication between the device information server and the client was implemented under `DevInfoNode` at `vehicle_interfaces/devinfo.h`. The client device can easily communicate with the server by inheriting the `DevInfoNode` class or the derived `VehicleServiceNode` class.

**NOTE:** To enable the funcitons of `DevInfoNode`, make sure to pass the correct service name to the `DevInfoNode` constructor. If user want to disable the functions, set the service name to empty string.

For the client device, there are several functions to communicate with the device information server:
- `regDevInfo()`: Register the device information to the server by manual.

**NOTE:** If the `DevInfoNode` was enabled, the node will run the system command to get the hostname, IP address, and MAC address. Since the information obtained, the node will try to register the information to the server until the information is successfully registered.



## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
    Install ROS2 from official website: [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) or simply run the following command to automatically install ROS2:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the Ubuntu version.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` or `foxy` for global ROS2 environment setup (e.g. `source /opt/ros/<$ROS_DISTRO>/setup.bash`) depending on the ROS2 version.
- [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git)
- nlohmann-json3-dev

The required packages are listed in the `requirements_apt.txt` file. Install the required packages by running the following command:
```bash
xargs sudo apt install -y < requirements_apt.txt
```
**NOTE:** The required packages will be installed automatically while installing the package using the (`vcu-installer`)[https://github.com/cocobird231/RV1-vcu-install.git].


### Installation
There are two ways to install the package: manually or using the `vcu-installer`. 

#### Install Manually
1. Check if `vehicle_interfaces` package is installed. If not, install the package by following the instructions in the [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git).
2. Clone the repository under `~/ros2_ws/src` and rename it to `cpp_devinfoserver`:
    ```bash
    git clone https://github.com/cocobird231/RV1-devinfoserver.git cpp_devinfoserver
    ```
3. Change the directory to the `~/ros2_ws` workspace and build the package:
    ```bash
    # Change directory to workspace.
    cd ~/ros2_ws

    # Source the local environment.
    . install/setup.bash

    # Build the package.
    colcon build --symlink-install --packages-select cpp_devinfoserver
    ```
    **NOTE:** The package is installed in the local workspace.


#### Install Using `vcu-installer`
1. Run the installer and press `Scan` button under Package Management window. If the installer not installed, install the installer by following the instructions in the [`vcu-installer`](https://github.com/cocobird231/RV1-vcu-install.git).

2. Checked the `DeviceInfo Server` checkbox under package list, right-click to modify the internet setting, then press the `Install` button to install the package.

3. The installer will create the start-up script for the package under `/etc/xdg/autostart` directory. The package will be started automatically after the system boot-up.


## Usage
The package contains two executables: `server` and `control`. The `server` executable is used to run the main service, while the `control` executable is used to control the service.

### Run the Main Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the main service:
    - Using the `launch`:
        ```bash
        ros2 launch cpp_devinfoserver launch.py
        ```
        **NOTE:** The launch file parsed the `common.yaml` file to set the parameters. The `common.yaml` file is located in the `cpp_devinfoserver/launch` directory.
        **NOTE:** The `common.yaml` file default the namespace to `V0`.

    - Using the `run`:
        ```bash
        ros2 run cpp_devinfoserver server
        ```

The service will create the JSON file for each registered node under the `devInfoServerDirPath` directory defined in the `common.yaml` file. If new registered node is conflict with the existing node, the service will update the latest information, and dump it to the JSON file. For the conflicted file, the service will create the backup file with the `.conflict` extension.

### Control the Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the control service:
    ```bash
    ros2 run cpp_devinfoserver control
    ```
    **NOTE:** If the main service is using the namespace (e.g. `V0`), the control service should use the same namespace to control the main service:
    ```bash
    ros2 run cpp_devinfoserver control --ros-args -r __ns:=/V0
    ```


## Description

### Communicate with the Service
The control executable demonstrates the communication between the client and the server using `DevInfoReg.srv` and `DevInfoReq.srv` which are defined in the `vehicle_interfaces/srv`. The service is used to register the device information to the server and request the device information from the server.

#### `DevInfoReg.srv`
The service is used to register the device information to the server. The service contains the following fields:
```.srv
# Request field
DevInfo dev_info # The device information to be registered.

# Response field
bool response # Whether the service is successfully executed.
```
The `DevInfo` message stores the device information, including the node name, hostname, IP address and MAC address. The message is defined in the `vehicle_interfaces/msg_content/DevInfo.msg` file, which contains the following fields:
```.msg
# Node name
string node_name

# Host name
string hostname

# MAC address
string mac_addr

# IPv4 address
string ipv4_addr

# IPv6 address
string ipv6_addr

# Multi-node
bool multi_node 0
```

#### `DevInfoReq.srv`
The service is used to request one or more device information from the server. The service contains the following fields:
```.srv
# Request field
DevInfo dev_info # The device information to be requested.

# Response field
bool response # Whether the service is successfully executed.

DevInfo[] dev_info_vec # The device information vector.
```
**NOTE:** In request field, fill any one of the fields of `dev_info` except `multi_node` field. The server will return the device information that matches the request field.

**NOTE:** In request field, if the field `node_name` of `dev_info` is set to "all", the server will return all the device information.


### `common.yaml` File
The `common.yaml` file is currently not used. The service name will be determined by the `devInfoService` under `service.json` file.
