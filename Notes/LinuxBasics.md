# Linux Basics
This is my guide that I put together to understand the basics of Linux.  As I was learning the operating system and command line, I found several existing guides for the OS, but they were written from the perspective of a master teaching a student.  These notes were started as I was learning Linux and should focus on the core of what is needed to navigate the system, install, and perform high-level development.

If you are viewing this, hopefully you will find this document useful.  They are not 100% complete, but should be good enough to jog the memory.

## Navigation Commands

`cd` change directory
- / root
- ~ home
- ..level up

`ls` list directory contents
-  -a ALL (including invisibles)

`mv` move file/directory [source] [destination]

`cp` copy file/directory [source] [destination]

`mkdir` make directory [name]

## Installing Packages
To install a package from the package manager, use `apt-get`:
1. Update the system and repositories
1. Install the desired package.
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install [Package Name]
```

If the package is not in any of the repositories used by `apt-get`, you will need to install the package from source.

For a `.deb` file, you will need to do the following:
1. Download and locate the package.  For future reference, I keep all manually downloaded and installed items in a folder `~/Install`
1. Unpack the compressed file.
2. Install
```
sudo dpkg -i /path/to/deb/file
sudo apt-get install -f
```
https://unix.stackexchange.com/questions/159094/how-to-install-a-deb-file-by-dpkg-i-or-by-apt

## Building Libraries/Files
1. Download the library of interest
1. Enter the directory
1. Create a build directory (if it does not already exist)
1. Use cmake to build the directory

```sh
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

Next use make/install to place library in `/usr/local/lib/`

``` sh
make
sudo make install
```

If make fails... fix the problem, then:

``` sh
make clean
make
```
For make, you can chose the number of cores used with the flag `-j#` where # is the number of cores.  For a quad core machine, the flag would be `-j4`.

## iStat Menus Equivalent
`sudo apt-get install indicator-multiload
`
https://askubuntu.com/questions/30334/what-application-indicators-are-available/40019#40019

## Disk Management and File Formats
The native Linux file format `ext4` is not compatible with other systems.  While `FAT16/32` is compatible with other operating systems, there is a 4GB limit to a file size.  This makes it difficult to transfer large files.  Therefore, I recommend formatting all USB drives as `exFAT`.

However, Ubuntu does not support `exFAT` by default and the proper tools will need to be installed.

```sh
sudo apt-get install gparted
sudo apt-get install exfat-utils exfat-fuse
```

The steps are as follows:
1. Backup/save all files on the drive.  The drive in question will be formatted and all files will be removed.
1. Open up Gparted and remove the existing file partition
1. Open up a terminal and type `sudo fdisk -l` to identify the disk to format
1. Run the command `sudo mkfs.exfat -n LABEL /dev/sdXn` to format the drive.  `LABEL` is the desired name of the new partition and `/dev/sdXn` is the partition id from the previous step.

**NOTE:** Gparted _cannot_ format a drive into `exFAT`.  

## Fix WiFi on Apple MacBook Pro with Broadcom Chip
From [askubuntu](https://askubuntu.com/questions/470153/no-wireless-when-install-14-04-on-macbook-pro):

```sh
sudo apt-get update
sudo apt-get install firmware-b43-installer
```
> Reboot if it does not connect post a new file from the script.

> No internet:

> Download the b43updated.zip file to a usb flash drive then drag and drop the file to your ubuntu desktop. Right-click it and select Extract Here.

> Open a terminal and do:
```
sudo mkdir /lib/firmware/b43
sudo cp Desktop/b43/*  /lib/firmware/b43
sudo modprobe -rv b43
sudo modprobe -v b43
```
> if it does not come on reboot.
