# NoeticLean/optitrack_motive_2_client

This package handles interfacing with the Optitrack Motive software in the motion capture room. It is a Noetic port of the package built by MIT-AERA (https://github.com/mit-aera/OptiTrack-Motive-2-Client).

## Motion Capture room setup

To connect to the robot remotely, a router and two ethernet cables are needed. An ethernet to USB adapter may be needed if the laptop being used does not have an ethernet port.

Power the router and connect one LAN port to the router in the MOCAP room labeled "OPTITRACK OUTPUT" and one LAN port to your machine. Optionally, the WAN port can be connected to the router labeled "INTERNET" for simplicity, as without this, your machine will not be able to access the internet while connected to the Optitrack network.

Login to the desktop labeled "MOCAP" using credentials:
* user: LEAN
* pass: mit-lean

Open the Motive client on the desktop. Go to File>Open and navigate to the `C:\Users\mocap\Documents\OptiTrack\Calibration`, and sort by Date Modified. Select a calibration labeled 'HalfSpace' with a temperature close to the current temperature listed by the [MIT Sailing Pavilion](https://sailing.mit.edu/weather/), but also one that is as recent as possible. Also, select View>'Data Streaming Pane', and ensure the Local Interface is set to `192.168.1.12`.

Turn on the MOCAP room cameras by turning on the labeled power strip in the corner of the room. It will take a minute or two to start up completely. 43 total cameras should appear on the right ahnd pane labeled 'Devices' in the Motive client, and any markers visible in the space should appear. Lights can be controlled near the door. Once the robot is placed in the MOCAP space, select their markers, right click and select 'Rigid Body'>'Create From Selected Markers'. Rename the body in the bottom right properties pane. Change the minimum marker count and max deflection as necessary.

> :warning: Be sure to delete the created rigid body once you are done testing in the MOCAP room, as a courtesy to others that use the space. Also **make sure that the room is left clean as it was found, the nets are closed off, the cameras are turned off, the lights are turned off, and all supplies you brought in are taken out.**

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

The netplan configuration can be applied with `sudo netplan apply`. The output of `ip a` should show

![Netplan configuration from output of ip a](/media/network.png)

After this, ensure that the host computer's `ROS_IP` environment variable is set to its own IP on the LEAN network using
```bash
export ROS_IP=$(hostname -I | cut -d' ' -f1)
```

If this doesn't work, ensure the machine is connected to the LEAN network. Use `hostname -I` to find the IP that is not `192.168.1.112` and then use that IP as in `export ROS_IP=127.0.0.1`.



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
