# NoeticLean/optitrack_motive_2_client

This package handles interfacing with the Optitrack Motive software in the motion capture room. It is a Noetic port of the package built by MIT-AERA (https://github.com/mit-aera/OptiTrack-Motive-2-Client).

## Motion Capture room setup

TODO

## Network setup

This setup assumes Ubuntu 20.04 or future. Note that the LEAN lab laptop already has this setup completed, and only has to be redone when a new ethernet adapter is used.

Ensure that the ethernet cable is connected to the host computer (via USB adapter if need be), and that the Motive client on the motion capture desktop has been set up properly. First, to identify the name of the ethernet interface, you can use `ip a` to get all interface names, then disconnect the ethernet cable and use `ip a` again, to figure out which one the ethernet is.

After this, a netplan configuration file must be made. A configuration file can be opened using nano:
```bash
sudo nano /etc/netplan/02-lean-network.yaml # requires sudo
```
and the following must be placed inside:
```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enx00e100001146:
      dhcp4: no
      addresses:
      - 192.168.1.112/24
      routes:
      - to: 239.255.42.99/4
        scope: link
```
replacing `enx00e100001146` whatever the name of your ethernet interface is.

The netplan configuration can be applied with `sudo netplan apply`.

After this, ensure that the host computer's `ROS_IP` environment variable is set to its own IP on the LEAN network using `export ROS_IP=192.168.50.84`, replacing the IP with the host machine's IP.

## Launching the client

Assuming the room and network have been properly setup, this client can be launched with:
```bash
roslaunch optitrack_motive_2_client optitrack_lean.launch
```
To make sure it worked, you can launch the rviz file in the `controller` package and see if the robot appears:
```bash
roscd controller
cd rviz
rviz -d frames.rviz
```
