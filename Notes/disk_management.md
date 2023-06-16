# Disk Management

## Check Available Disk Space

```sh
# Check the space avaible on the system
df -h

# Check the size of a specific directory
du -sh <dir_name>
```

## Getting Device Information

```sh
# List all of the availble disks on a system
sudo lsblk

# Get specific imformation about a particular disk
sudo fdisk -l /dev/sdb
```

**Recall:** the filesystem will mount disks in sequential order `sda`, `sdb`, `sdc`, etc.

## The GUI approach

If you have a GUI, the tool `gparted` provides an excellent interface for managing disks.

```sh
# Install
sudo apt install gparted -y
```