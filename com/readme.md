# CAN 聊天软件

### On Ubuntu

1. Install dependent packages:

```bash
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -y can-utils net-tools
```

2. Check whether can device is detected:

```bash
ifconfig -a
# If a device with can name is found in the list, it means that the device can be recognized
```

3. Connect the two can ports with a **straight line** to facilitate loop back test.
4. Initialize CAN0 and CAN1 devices with a bit rate of 1Mbits:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```
