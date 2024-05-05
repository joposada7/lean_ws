# Network Setup

This document details the configuration needed to get the network connected between the host machine and the Raspberry Pi.

> :shipit: **This setup is primarily for the 2.4 GHz network interface on the Pi. In order to configure the Pi to use an external 5 GHz network adapter and connect via that, use the [5g_network_setup](./5g_network_setup.md) document.**

## Router

The current router being used in LEAN is the ASUS AC1750 (RT-AC65), configured with the following credentials:
* SSID (2.4 GHz): LEAN
* SSID (5 GHz): LEAN-5G
* pass: leanpass

The router is set to a local network on 192.168.50.x. The admin site can be accessed via default gateway 192.168.50.1 with credentials:
* user: admin
* pass: ceimp

The router can also be connected to the internet using an ethernet cable in its WAN port (blue) for ease of use.

## Connecting to the Pi

The Raspberry Pi Zero 2 W by default only supports 2.4GHz connections. Its network should be configured at the time the Debian buster OS is loaded onto its SD card using the Raspberry Pi Imager settings, but can otherwise be manually configured as detailed in [Manually configuring the Pi network](#Manually-configuring-the-Pi-network).

The username of each Pi should also be set at the time the OS is loaded onto the SD card. The IP address of each Pi is assigned by DHCP and can be found by viewing the list of clients on the router at its admin site. With current (5/5/2024) configurations, each machine can be accessed over the LEAN (2.4 GHz) network using:
```bash
ssh shrimp@192.168.50.81 # car
ssh wayfarer@192.158.50.105 # boat
ssh bean@192.168.50.36 # blimp
```

Additionally, passwordless SSH can be set up by running the following command on the host machine:
```bash
ssh-copy-id shrimp@192.168.50.881
```

Adjusting the user and IP for each machine.

### Setting /etc/hosts

In order to remotely launch nodes using ROS, the `/etc/hosts` file on both machines must be properly configured. Simply edit the file on the host machine to include:
```
192.168.50.81 shrimp
192.168.50.105 wayfarer
192.168.50.36 bean
```

And edit the file on each Pi to include:
```
192.168.50.100 myhostuser
```

Adjusting the IP to reflect your host IP (found using `hostname -I` on the host machine) and the hostname (found using `hostname` on the host machine).

### Manually configuring the Pi network

If for some reason, the Pi does not connect to the LEAN network automatically, or the network must be configured, the user can manually edit the `wpa_supplicant-wlan0.conf` file.

You can access the file by either removing the SD card and manually accessing the `/etc/wpa_supplicant/` directory, or by connecting to the Pi via an ethernet connection.

If you choose to use an ethernet connection, you will need to connect an ethernet cable between the Pi and the host machine using an Ethernet to micro-USB adapter. After this,

> :warning: **TODO: Finish this section with how to set up IP address to SSH into Pi over ethernet.**

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