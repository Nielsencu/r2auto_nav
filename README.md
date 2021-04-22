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

	If you check your permissions again:

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

#Setting up of RPis for Turtlebot


	
	
	


