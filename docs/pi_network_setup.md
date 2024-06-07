# Pi Network Setup

This document details the configuration needed to get the network connected between the host machine and the Raspberry Pi.

> :shipit: **This setup is primarily for the 2.4GHz network interface on the Pi. In order to configure the Pi to use an external 5GHz network adapter and connect via that, use the [5g_network_setup](./5g_network_setup.md) document.**

## Router

The current router being used in LEAN is the ASUS AC1750 (RT-AC65), configured with the following credentials:
* SSID (2.4 GHz): LEAN
* SSID (5 GHz): LEAN-5G
* pass: leanpass

The router is set to a local network on 192.168.50.x. After connecting to the network, the admin site can be accessed via default gateway 192.168.50.1 with credentials:
* user: admin
* pass: ceimp

The router can also be connected to the internet using an ethernet cable in its WAN port (blue) for ease of use, although this is not necessary to connect and control the Pi.

## Connecting to the Pi

The Raspberry Pi Zero 2 W by default only supports 2.4GHz connections. Its network should be configured at the time the Debian buster OS is loaded onto its SD card using the Raspberry Pi Imager settings, but can otherwise be manually configured as detailed below in [Manually configuring the Pi network](#Manually-configuring-the-Pi-network).

The username of each Pi should also be set at the time the OS is loaded onto the SD card. The IP address of each Pi is assigned by DHCP and can be found by viewing the list of clients on the router at its admin site. With current (5/5/2024) configurations, each machine can be accessed over the LEAN (2.4 GHz) network using:
```bash
ssh shrimp@192.168.50.81 # car
ssh wayfarer@192.158.50.105 # boat
ssh bean@192.168.50.36 # blimp
```

Set up passwordless SSH by running the following command on the host machine. This is needed to properly remotely launch nodes in ROS:
```bash
ssh-copy-id shrimp@192.168.50.81
```
adjusting the user and IP for each machine.

### Manually configuring the Pi network

If for some reason the Pi does not connect to the LEAN network automatically, or the network must be configured, the user can manually edit the `wpa_supplicant-wlan0.conf` file.

You can access the file by either removing the SD card and manually accessing the `/etc/wpa_supplicant/` directory on a computer, or by connecting to the Pi via an ethernet connection. Manually accessing the file on a computer is easier in most cases, although an ethernet connection may be established as detailed below in [Connecting to the Pi via Ethernet](#Connecting-to-the-Pi-via-Ethernet).

Edit the `wpa_supplicant-wlan0.conf` as follows, or adjust for any new SSID and password:
```yaml
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

country=US

network={
        ssid="LEAN"
        psk="leanpass"
}
```

Then reboot the system.

### Connecting to the Pi via Ethernet

In some cases, it is beneficial to maintain an ethernet connection to the Pi without WiFi or a router.

To set this up, you will need to either be able to edit files directly on the Pi's SD card, or a preexisting WiFi connection to the Pi. First, connect an ethernet cable between the Pi and the host machine using an ethernet to micro-USB adapter. After this, IPs must be manually assigned to both the host machine and the Pi for the ethernet interface.

On the host computer, a netplan configuration can be set up to assign an IP address on the ethernet interface by placing this within a file `/etc/netplan/03-lean-ethernet-network.yaml`:
```yaml
network:
    version: 2
    renderer: NetworkManager
    ethernets:
        enxf8e43baaa8bf:
            dhcp4: no
            addresses:
            - 192.168.1.3/24
            routes:
            - to: default
              via: 192.168.1.1
            nameservers:
                addresses:
                - 192.168.1.1
```
adjusting the ethernet interface `enxf8e43baaa8bf` with the appropriate name as shown in the output of `ip a`, then running `sudo netplan apply`.

On the Pi, the `/etc/dhcpcd.conf` file should be edited to include at the top of the file:
```
interface eth0
static ip_address=192.168.1.2/24
static router=192.168.1.1
static domain_name_server=8.8.8.8
```
then reboot the Pi.

The Pi can then be connected to via SSH over the ethernet connection by using the newly applied IPs, with `ssh user@192.168.1.2`, with the appropriate username.