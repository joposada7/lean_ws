# lean_ws/optitrack_motive_2_client

This package handles interfacing with the Optitrack Motive software in the motion capture room. It is a Noetic port of the package built by MIT-AERA (https://github.com/mit-aera/OptiTrack-Motive-2-Client).

## Motion Capture room setup

To connect to the robot remotely, a router and two ethernet cables are needed. An ethernet to USB adapter may be needed if the laptop being used does not have an ethernet port.

Power the router and connect one LAN port to the router in the MOCAP room labeled "OPTITRACK OUTPUT" and one LAN port to your machine. Optionally, the WAN port can be connected to the router labeled "INTERNET" for simplicity, as without this, your machine will not be able to access the internet while connected to the Optitrack network.

Login to the desktop labeled "MOCAP" using credentials:
* user: LEAN
* pass: mit-lean

Open the Motive client on the desktop. Go to File>Open and navigate to the `C:\Users\mocap\Documents\OptiTrack\Calibration`, and sort by Date Modified. Select a calibration labeled 'HalfSpace' with a temperature close to the current temperature listed by the [MIT Sailing Pavilion](https://sailing.mit.edu/weather/), but also one that is as recent as possible. If no suitable calibration file is available, reference the dropdown below for calibrating them yourself. Also, select View>'Data Streaming Pane', and ensure the Local Interface is set to `192.168.1.12`.

<hr/>
<details>
  <summary><i>MOCAP Cameras calibration</i></summary>
  If there is no suitable calibration (e.g. they are over 1 month old, or >10F in temperature difference), then you can calibrate the cameras yourself. The room comes with the following items for calibration:
  <ul>
    <li>
      Calibration wand hot glued on top of drone battery <br/>
      <img src="/media/calibration_wand.png" alt="Calibration wand hot glued onto drone battery in MOCAP room." width="400"/>
    </li>
    <li>
      Drone controller <br/>
      <img src="/media/drone_controller.png" alt="Controller used for the calibration drone in MOCAP room." width="400"/>
    </li>
    <li>
      Calibration drone <br/>
      <img src="/media/calibration_drone.png" alt="Calibration drone used in MOCAP room." width="400"/>
    </li>
  </ul>
  Make sure the drone battery and the drone controller are both charged. Slide a table under the net inside the motion capture space in order to place the drone on inside the net for takeoff.

  <img src="/media/table_in_space.png" alt="Rolling table placed under the safety net inside the motion capture half-space." width="400"/>

  Turn on the MOCAP room cameras if they are not already on by turning on the labeled power strip in the corner of the room. It will take a minute or two to start up completely. In the Motive client, you should see a "Camera Calibration" pane on the left-hand side. Select Calibration Type to be "Refine" and the OptiWand to be "CW-500 (500mm)", as per the calibration wand model.

  <img src="/media/calibration_pane.png" alt="Camera Calibration asset pane in the OptiTrack Motive client." width="400"/>

  Attach the drone battery and calibration wand to the drone, then place the calibration drone inside the space, on top of the table (but inside of the net) to prepare for takeoff. In the Motive client, hit "Start Wanding", and then begin flying the drone around the space.

  <img src="/media/drone_ready_to_fly.png" alt="Calibration drone placed inside the net on top of the table and ready to fly." width="400"/>

  The goal of calibration is to hit around an equal number of samples per camera (although pratically, this can be difficult), and to hit around 10,000 samples each. The number of samples each camera has collected can be seen in the Motive client. Note that there is no need to fly too high if you will not be conducting experiments that high, although you will want samples to vary in altitude.

  > :warning: **The calibration wand makes the drone quite unstable! It will oscillate back and forth while flying. Ensure that the net is secure BEFORE flying in case you crash. Do NOT fly it too quickly, and make sure to maintain line of sight AT ALL TIMES.**

  After landing the drone, hit "Calculate" in the Motive client. Several metrics will appear and can be evaluated as needed for any experiments to be run. Note that a typical mean 3D error for this method of calibration should be around 0.800-0.900 mm. If it is much higher (>2.00 mm), consider redoing the calibration. Select "Apply", and then enter the "Ground Plane" tab on the left-hand pane.

  <img src="/media/calibration_result.png" alt="Camera calibration results and metrics displayed with exceptional result and mean errors." width="400"/>

  Remove the calibration drone from the space, and disconnect its battery. At this point, you are able to put the drone battery and drone controller back to charge near the power strip. Place and turn on the ground plane calibrator at the origin of the motion capture space. The origin is located somewhere on the middle of the floor beneath the net, and is marked with axes and indents for the markers of the ground plane to be placed.

  <img src="/media/ground_plane.png" alt="Ground plane calibrator with 9V battery." width="400"/>

  In the Motive client, hit "Set ground plane". At this point, calibration should be done and the new file should be generated. The ground plane inside the space should be turned off and removed from the space. Name this file appropriately, following the naming convention: **Calibration [Quality] HalfSpace-[Temperature] (MeanErr [MeanErr]) [YMD Date].cal**. An example is "Calibration Exceptional HalfSpace-67F (MeanError 0.890 mm) 2023-10-06.cal". The temperature should be taken from the <a href="https://sailing.mit.edu/weather/">MIT Sailing Pavilion website</a>.

  > :warning: **DON'T FORGET: Charge the drone controller and calibration stick once done calibrating. Turn off the ground plane calibrator and remove it from the space.**
</details>
<hr/>

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
