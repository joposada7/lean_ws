# 5GHz Network Setup

This document details the configuration needed to get the 5GHz network interface working on the Pi.

> :shipit: **This setup is only to configure the Pi to use a 5GHz network. In order to connect to the Pi using 2.4GHz WiFi or ethernet, reference the [pi_network_setup](./pi_network_setup.md) document.**

## 5G Adapter

The Pi needs to use a Linux-compatible 5GHz WiFi adapter connected via a micro-USB adapter. Currently, a TP-Link AC600 Archer T2U Nano and a USB to micro-USB adapter is used on the car robot because of weight issues. Note that in the future, it may be possible to include a USB port directly into the PCB of each robot to eliminate the need to use a heavy micro-USB adapter.

<img src="/media/car_5g_adapter.png" alt="Car robot pictured with TP-Link AC600 Archer T2U Nano adapter connected via a USB to micro-USB adapter." width="400"/>

The appropriate drivers for the adapter must then be installed on the Pi before it can be used to connect to a network. First, check the device ID using `lsusb`, which outputs like:
```
$ lsusb
Bus 003 Device 004: ID 2357:011e TP-Link 802.11ac WLAN Adapter
```
where the device ID is `2357:011e`. This can be used to find the most up-to-date Linux drivers online. As of writing (6/7/2024), the following can be used to install the correct drivers for the Archer T2U Nano on the Pi ([askubuntu reference](https://askubuntu.com/questions/1149117/tp-link-ac600-archer-t2u-nano-driver-for-ubuntu-18-04)):
```bash
sudo apt install git dkms
git clone https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au
sudo make dkms_install
```
then reboot.

## wpa_supplicant configuration

In order to connect the Pi to a 5G network, the wpa_supplicant files must be configured. By default, the Pi has a file `/etc/wpa_supplicant/wpa_supplicant.conf` that should have been generated at boot after flashing the SD card for the first time, which defines the network to connect to. To configure a 5G network, we configure the `wlan0` and `wlan1` interfaces separately ([superuser reference](https://superuser.com/questions/469039/wpa-supplicant-for-dual-wlan)) by placing this in `/etc/wpa_supplicant/wpa_supplicant-wlan0.conf`:
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

country=US

network={
        ssid="LEAN"
        psk="leanpass"
}
```
and this in `/etc/wpa_supplicant/wpa_supplicant-wlan1.conf`:
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

country=US

network={
        ssid="LEAN-5G"
        psk="leanpass"
}
```
adjusting for any changes to the network SSIDs or password.

## Using the 5G network

It appears that when both are available, the Pi will prefer to use the first wlan (2.4GHz) configuration. In order to force the Pi to use the 5G network, the first interface must be disabled on the Pi using:
```bash
sudo ifconfig wlan0 down
```

This will hang the SSH session if you were connected via the 2.4GHz IP. You can find the new 5G IP through the router admin site as described in the [pi_network_setup](./pi_network_setup.md) document. You can then connect to the Pi using this IP over SSH as normal.

> :warning: **Note that this change of IP address must be reflected in the host machine /etc/hosts file or ROS programs will not work. Refer to the [ros_setup](./ros_setup.md) document.**