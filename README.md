# Group 8 Autonomous Navigation Turtlebot With IR Detecting Ping-Pong Cannon

Group 8 has recorded the steps required to get an up-and-running turtlebot 
required for autonomous navigation,mapping,detection via IR,aiming and firing of
ping pong ball(Versions of repos and libraries mentioned are 
the most updated at the time this documentation is written. Any further releases
may not be entirely supported)




## Installation Of Required OS and Packages

Using the IMG file provided, follow the instructions in the link provided to get your OS up.

The OS required is the Ubuntu 20.04.2.0 LTS Desktop (64 bit) 

IMG:(https://releases.ubuntu.com/20.04/)
Installation Instructions:(https://ubuntu.com/tutorials/install-ubuntu-desktop#1
-overview)
 

Using the bash terminal, we can now install ROS 2 Foxy Fitzroy

```bash
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
sudo chmod 755 ./install_ros2_foxy.sh
bash ./install_ros2_foxy.sh
```

#Install ROS 2 dependencies: Gazebo11, Cartographer and Navigation2

```bash	
sudo apt-get install ros-foxy-gazebo-*

sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros

sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```

#Install Turtlebot3 Packages

```bash
source ~/.bashrc
sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3
```

## Setting up your ROS Environment

ROS uses the Data Distribution Service (DDS) to communicate. Your Domain ID 
does not have to follow ours but it is best to to get the most optimal results.


```bash
echo 'export ROS_DOMAIN_ID=37 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```
	
#SSH Setup (With AWS)
We will primarily be using SSH to access the Rpi. For this, it is optimal to have our local network set up with a static ip address. However, we decided to
use a virtual machine (VM) set up on an Amazon Web Server (AWS) due to DHCP in the school network.

Sign up for a free-tier AWS account: https://console.aws.amazon.com/console/home?nc2=h_ct&src=header-signin

Click “EC2” under “AWS services”.

Click the “Launch Instance” button.

Select “64-bit (x86)” option under “Amazon Linux 2 AMI (HVM), SSD Volume Type”.

Make sure the “t2.micro” instance type is selected, and then click “Review and Launch”.

Click the “Launch” button.

When the key pair window comes up, select the “Create a new key pair” under the first drop-down menu.

Enter a name for your key pair (e.g. MyKeyPair), and then click “Download Key Pair” (A .pem file will be downloaded, which you can use to access your AWS account in the future)

Click the “Launch Instances” button.

While waiting for the instance to launch, you can click the “Services” button in the top-left of the window and select “EC2”. This will take you to the EC2 Dashboard.

Click on the “Instances (running)” button, which will show you information about your running instances. There should only be one entry at this point, so you can click on the instance under the “Instance ID” column. This should show you information about your instance, 

including the public IPv4 address (Note this down) 

You will need to change the permission settings on the key pair file, so first open up the Terminal application and check your file permissions:

```bash	
ls -l ~/Downloads/MyKeyPair.pem
```

This should return the following:

```bash
-rw-r--r--@ 1 syen  staff  1700 Dec 13 22:18 /Users/syen/Downloads/MyKeyPair.pem
```

indicating that the owner (i.e. you) has read and write permissions, but users in your group only have read permissions, and other users only have read permissions. AWS will reject these permissions as not being secure enough, so you will need to change the permissions by doing:

```bash
chmod 400 ~/Downloads/MyKeyPair.pem
```

If you check your permissions again:


```bash
ls -l ~/Downloads/MyKeyPair.pem
```

you should see: 

```bash
-r--------@ 1 syen  staff  1700 Dec 13 22:18 /Users/syen/Downloads/MyKeyPair.pem
```

which means that only you have read permissions, and even the write permissions have been removed so you will not accidentally modify the file.




In order to simplify the login process, create a SSH public-private key pair on your laptop:

```bash
ssh-keygen
```

Hit Return three times to use the default file name, and to accept a blank password. This will create two files in the .ssh directory in your home directory: id_rsa and id_rsa.pub

You can now use the following command to copy the second file to your EC2 instance :

```bash
scp -i ~/Downloads/MyKeyPair.pem ~/.ssh/id_rsa.pub ec2-user@IP-address:~/
```

Login to your EC2 instance:

```bash	
ssh -i ~/Downloads/MyKeyPair.pem ec2-user@IP-address
```

You might get a warning that says:

```bash
The authenticity of host '52.221.221.183 (52.221.221.183)' can't be established.
ECDSA key fingerprint is SHA256:XRs+sZbEFRleJY8H41EH4nxRG1BYj4XO5+tKWCREzlk.
Are you sure you want to continue connecting (yes/no/[fingerprint])?

You should enter “yes”. This should show you the following login screen:

       __|  __|_  )
       _|  (     /   Amazon Linux 2 AMI
      ___|\___|___|

https://aws.amazon.com/amazon-linux-2/
7 package(s) needed for security, out of 19 available
Run "sudo yum update" to apply all updates.
[ec2-user@ip-172-31-33-20 ~]$ 
```

This means that you have successfully accessed your EC2 instance.




Run the ssh-keygen command to create keys on your EC2 instance:
	
```bash
ssh-keygen
```

Move the id_rsa.pub file from your laptop to the .ssh directory:

```bash
mv id_rsa.pub ~/.ssh/authorized_keys
```



You should now be able to login:

```bash
ssh ec2-user@IP-address
```


You can create an alias by editing the config file using nano or vim:

```bash
nano ~/.ssh/config
```

Enter the following (replace IP-address below with the IP address of your EC2 instance):
	
```bash
Host aws 
HostName IP-address
User ec2-user
```

You should now be able to login:

```bash
ssh aws
```

Be sure that you do not shut down or restart your AWS instance as it will change the IP address of the AWS instance, and you will then need to update the address in your scripts.
If you are working in a group and would like your group members to have access,they can do the following:
	
```bash
ssh-keygen
cd .ssh
cat id_rsa.pub
```

They can copy the contents of the id_rsa.pub and share it with you via email. You should then add the key into the authorized_keys file.
	
```bash
nano ~/.ssh/authorized_keys
```

### Setting up of RPis for Turtlebot

Download Ubuntu 20.04.1(Focal) Preinstalled Server image(http://cdimage.ubuntu.com/ubuntu-server/focal/daily-preinstalled/current/)
We need to burn the .img file to a microsd card.
Unzip the image
Use "Restore Disk Image" option in the "Disks" utility in Ubuntu
This will burn your .img file onto your microsd card
Insert your microsd card into the Rpi, connect a keyboard,external display and 
the power cable to their relevant ports.

### Configure your Rpi

Log in with default username(ubuntu) and password(ubuntu). After logged in, system will ask you to change the password.
Open automatic update setting file.
```bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades 
```
Disable automatic updates
```bash
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```
Save the file and exit

Run the following command to generate a hash for your password so that your password will not be stored in clear text (replace “password” with your password):
```bash
echo -n password | iconv -t utf16le | openssl md4
```
Note down the hash (the characters after the “= ” characters) 



To configure wifi settings:
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
Append the following:
```
	network:
	  version: 2
	  renderer: networkd
	  ethernets:
	    eth0:
	      dhcp5: yes
	      dhcp6: yes
	      optional: true
	      access-points:
		<WIFI_SSID>:
		   passwod: <WIFI_PASSWORD>
```
Save the file and exit

Apply netplan and reboot:
```bash
sudo netplan apply
sudo reboot
```
Set the systemd to prevent boot-up delay even if no network on startup.
```cli
systemctl mask systemd-networkd-wait-online.service
```
Disable Suspend and Hibernation
```cli
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```
Install and enable the SSH
```bash
sudo apt install ssh
sudo systemctl enable --now ssh
reboot
```
Once the pi boots up again, enter:
```cli
hostname -I
```
From your laptop, you can now ssh in. The external monitor and keyboard can be disconnected from the rpi
```bash
ssh ubuntu@<IP Address of RPi>
```
#Add Swap Space


Enter the command below to create 2GB swap space.
```cli
sudo swapoff /swapfile
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo nano /etc/fstab
```
You can ignore below error when entering swapoff /swapfile command.
```cli
swapoff: /swapfile: swapoff failed: No such file or directory
```
When the editor opens the fstab file, append below contents at the end of the file.
```cli
/swapfile swap swap defaults 0 0
```
Check if 2GB of swap space is correctly configured.
```cli
sudo free -h
```
You should see a table something like this:    

	          total        used        free      shared  buff/cache   available
	Mem:           912M         97M        263M        4.4M        550M        795M
	Swap:          2.0G          0B        2.0G


### Install ROS Foxy Fitzroy on RPi

Open the Terminal

Update and upgrade your software
```bash
sudo apt update && sudo apt upgrade
```
Setup locale
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
Setup sources
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
Install ROS 2 packages
```bash
sudo apt update
sudo apt install ros-foxy-ros-base
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
sudo apt install ros-foxy-hls-lfcd-lds-driver
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-dynamixel-sdk
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
source ~/.bashrc
colcon build --symlink-install --parallel-workers 1
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
### Environment Setup 

In DDS communication, ROS_DOMAIN_ID must be matched between Remote PC and TurtleBot3 for wireless communication under the same network environment.

A default ID of TurtleBot3 is set as 0.
To configure the ROS_DOMAIN_ID of Remote PC and SBC in TurtleBot3 to 30 is recommendable.
Use the following commands.
```bash
echo 'export ROS_DOMAIN_ID=37 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```
#OpenCR Setup

Connect the OpenCR to the Rasbperry Pi using the micro USB cable.
Install required packages on the Raspberry Pi to upload the OpenCR firmware.
```bash
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf
```
Depending on the platform, use either burger or waffle for the OPENCR_MODEL name. In our case, it was burger.
```bash
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
```
Download the firmware and loader, then extract the file.
```bash
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xjf ./opencr_update.tar.bz2
```
Upload firmware to the OpenCR.
```bash
cd ~/opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```
### Optimisations

we will remove the need to plug a monitor into your R-Pi in order to find out its IP address. We will do this by creating a script that will be run during startup to send the R-Pi’s IP address to your AWS instance.

We will first create a SSH public-private key pair that will allow it to ssh to AWS to update the IP address:
```bash
ssh-keygen
cat ~/.ssh/id_rsa.pub
```
In another Terminal window on your laptop, login to your group’s AWS account:
```bash
ssh aws
```
Edit the authorized_keys file using the nano editor:
```bash
nano ~/.ssh/authorized_keys
```
Paste the public key from the R-Pi into the file.

Exit and save the file.

To connect to AWS service, we need to edit the config file.
```bash
nano ~/.ssh/config
```
Enter the following into the file : 
```bash
host aws
HostName <IP-address>
User ec2-user
```
Exit and save the file.

We now need to create a system service that will run whenever the R-Pi starts up on both R-Pi’s:
```bash
sudo nano /etc/systemd/system/ip2aws.service
```
Enter the following into the file, which will run the ip2aws.bash script in the home directory when the R-Pi boots up:
```
	[Unit]
	After=network.service

	[Service]
	ExecStart=/home/ubuntu/ip2aws.bash

	[Install]
	WantedBy=default.target
```
We will now create the script that the service will run:
```bash
	nano ~/ip2aws.bash
```
Enter the following into the file for the R-Pi, which will get the IP address, remove the space character at the end, and use the ubuntu account to save the IP address on AWS in a file named “rpi.txt”:
```
	#!/bin/bash

	myIP=`hostname -I | tr -d " "`
	echo $myIP | runuser -l ubuntu -c 'ssh aws "cat - > rpi.txt"'
```

Change the permissions of the files
```bash
chmod 744 ~/ip2aws.bash
sudo chmod 664 /etc/systemd/system/ip2aws.service
```
Register the service
```bash
sudo systemctl daemon-reload
sudo systemctl enable ip2aws.service
```
Reboot the R-Pi
```bash
sudo reboot
```
Once the R-Pi’s have booted up, you should be able to check that the IP addresses are correctly saved.
```bash
ssh aws cat rpi.txt
ssh aws cat rpi2.txt
```
Edit the .bashrc file on your laptop to add the following at the end:
```bash
alias sshrp='ssh ubuntu@`ssh aws cat rpi.txt`'

alias sshrp2='ssh ubuntu@`ssh aws cat rpi2.txt`'
```
Reload the bashrc file:

```bash
source ~/.bashrc
```

Copy the ssh public key from your laptop to the R-Pi:
```bash
ssh-copy-id ubuntu@`ssh aws cat rpi.txt`
```


We will now add a few aliases and shell variables to the .bashrc on your laptop as well as the R-Pi’s to make it easier to get things running:

Add the following to the .bashrc on your laptop:

```bash
export TURTLEBOT3_MODEL=burger
alias rteleop='ros2 run turtlebot3_teleop teleop_keyboard'
alias rslam='ros2 launch turtlebot3_cartographer cartographer.launch.py'
```

Reload the .bashrc:
```bash
source ~/.bashrc
```
Add the following to the .bashrc on both R-Pi’s:

```bash
export TURTLEBOT3_MODEL=burger
alias rosbu='ros2 launch turtlebot3_bringup robot.launch.py'
```
Reload the .bashrc:

```bash
source ~/.bashrc
```


Once your project members are able to ssh in,change the password:
```bash
passwd
```
# Overview
Software flow for mapping and firing.

<p align="center">
	<img src="doc/soft_flow.png" width="750"/>
</p>

# Navigation
For the occupancy grid data, -1 value is mapped to 1, and occupancy grid values ranging from 0 to 60 are mapped to 2 and 60 to 100 are mapped to 3.

In this case, 1 is unmapped, 2 is free space, and 3 is blocked.

For navigation, Breadth First search is used to find unmapped areas. Breadth first search starts from robot's current position.

Once an unmapped area has been found, A* Search is used to find path between robot's current position and the unmapped area.

For visualization purposes, if path is found, it is published to /global_plan topic. One can use Rviz to visualize the path.



	
	
	


