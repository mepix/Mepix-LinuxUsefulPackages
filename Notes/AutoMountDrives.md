# Automatically Mounting Drives on Ubuntu

## Step 1: Get the Drive Info

Find the name, UUID, and format

```sh
sudo blkid
```

## Step 2: Make a Mounting Point

```sh
sudo mkdir /mnt/<name-of-the-drive>
```

## Step 3: Add to Config File

```sh
sudo gedit /etc/fstab
```

Append the following info to the file, with each item separated by the tab (`\t`) character:

```
UUID=<uuid-of-your-drive>  <mount-point>  <file-system-type>  <mount-option>  <dump>  <pass>
```

For example:

```
UUID=eb67c479-962f-4bcc-b3fe-cefaf908f01e  /mnt/sdb9  ext4  defaults  0  2
```

## Step 4: Reset System

Either reboot or run `sudo mount -a`

## Step 5: Make the Drive Mountable

```sh
cd <path>/<to>/<mount-point>
sudo chmod -Rv a+w .
```

## References

- [How To Automount File Systems on Linux](https://www.linuxbabe.com/desktop-linux/how-to-automount-file-systems-on-linux)
- [How to Make Disk Drive and Partitions Writable on Ubuntu](https://www.binarytides.com/access-other-partitions-as-writable-from-dolphin-kde-in-ubuntu/)
