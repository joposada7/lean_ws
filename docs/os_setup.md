# OS Setup

ROS Noetic officially supports Ubuntu 20.04 (Host machine) and Debian Buster (Raspberry Pi). Below are instructions on how to install these on your machine and on the Pi Zero 2 W.

## Host computer

Installation will require a USB drive, and a decent amount of storage on the machine (at least 25G, but more is preferable).

> :warning: **Installation of another OS on the host computer varies by machine, and it's always best to exercise caution as messing up the partitions or  installation can lead to deleting all data on the computer.**

A full-length tutorial for this can be found here: https://www.youtube.com/watch?v=-iSAyiicyQY.

### Partitioning a drive

This step will prepare space on either a HDD or an SDD on your machine to install Ubuntu. You can also follow these instructions: https://www.youtube.com/watch?v=9gS5SoogltE.

On Windows, go to the Start Menu and look for 'disk partition'. Here, you will see all disks installed on your machine and their partitions.

From here, select the drive and partition you'd like to partition to make space for Ubuntu, select 'Shrink Volume' and enter the amount of space you'd like to allocate. Do not convert this space to a Simple Volume.

### Creating a bootable drive

Download the Desktop LTS version of Focal here: https://releases.ubuntu.com/focal/.
Download Rufus here: https://rufus.ie/en/.

In the Rufus software, select the USB drive and the .iso Ubuntu image, then select START. This will delete all data on the USB and turn it into a bootable drive.

### Installing Ubuntu

Keep the drive inserted into the machine, and restart the computer. During boot-up, continually press F12 to enter the boot menu. Note that on some computers, the designated key may be F2, F5, F8, F9, or another key.

In the boot menu, look for 'Boot from USB' or 'Boot Ubuntu'. Some drives may show as their brand (e.g. as SanDisk). This option may be in advanced or boot settings. Once Ubuntu boots, click 'Install Ubuntu' and proceed with the installation. Typically, it's best to go with the 'Minimal Installation' option to preserve space.

On the 'Installation type' window, select 'Something else' in order to select the partition you previously made. You should see it listed as 'free space' under the correct drive.

> :warning: **Be careful on this step! Selecting the wrong drive or partition WILL format it and delete all data you had there, including any OS booting from there.**

Click the + button and create a Logical partition as an Ext4 journaling file system mounted at the root directory `/`, but leave about 4-8GB. Use the final space to create a Logical partition as swap area. Finally, **select the Device for boot loader installation as the partition you create for the root directory `/`**.

From here, you can proceed with the installation as normal. Upon full installation, you will be able to restart the machine, remove the USB drive, and see both your original OS and Ubuntu 20.04 in the GRUB bootloader menu.

## Raspberry Pi

Install the Raspberry Pi Imager from this link: https://www.raspberrypi.com/software/ 
Download the Debian Buster ISO: https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/2021-05-07-raspios-buster-arm64.zip

### Flashing the microSD

Insert your microSD and choose the custom .iso option, then choose the above downloaded image. 

Open the advanced options by pressing the gear at the bottom right and enable SSH, setting a personal username and password (name of your platform as username, "pass" as the password). Then configure wireless LAN to the following - SSID: LEAN, Password: leanpass - and save the options. 

Flash your microSD card and wait for it to complete.

Remove your microSD then put it back into your computer so that you can access the files. Find the file named "bcm2710-rpi-3-b.dtb", copy it, then rename the copy to "bcm2710-rpi-zero-2.dtb".

You can now boot up and SSH into your Pi.

### Configuring the Raspberry Pi

Once the Pi is booted up, enter the following commands:
```bash
sudo apt-get update --allow-releaseinfo-change
sudo apt-get update
sudo apt-get remove --purge vlc
sudo apt-get autoremove
sudo apt-get clean
sudo apt-get upgrade
```

We will now reconfigure our locale:
```bash
sudo raspi-config
```

Choose Localisation Options --> Locale --> All locales --> en_GB.UTF-8 --> Finish

We will now increase the swap file size.

```bash
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
```

Edit the swap file size, increase CONF_SWAPSIZE to 2048, save and exit.

Create, initialize, and start the swap file.
```bash
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

If preferred, Git can be installed on the Pi and configured with an SSH key tied to your personal Github account. To do this, see [How to generate a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [How to add a new SSH key to Github](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=windows).  However, if you want to avoid this, files can be directly sent from the machine to the Pi using rsync:
```bash
rsync -av ~/lean_ws/src/ shrimp@shrimp:~/lean_ws/src/ # On host machine
```

The Raspberry Pi is now ready to install ROS.