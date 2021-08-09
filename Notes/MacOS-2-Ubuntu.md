# Installing Ubuntu 18 on a 2011 Macbook Pro

## Create a Bootable Image
Download the appropriate version of [Ubuntu](https://www.ubuntu.com), then create a bootable image using [Etcher](https://www.balena.io/etcher/).

Insert the image into the Macbook Pro and hold the `option` key while booting. Select the EFI bootable disk.

## Issues with `nomodeset`
I ran across some issues with the `nomodeset` flag. A complete set of insturctions on how to resolve this issue can be found [here](https://sts10.github.io/2016/11/08/installing-ubuntu-on-my-old-macbook-pro.html).


- When the boot menu [appears](https://askubuntu.com/questions/38780/how-do-i-set-nomodeset-after-ive-already-installed-ubuntu), press the `e` key to edit the boot options. This opens an options file in a basic text editor. Find the line that has `quiet splash` in it and make that bit of the line read `nomodeset quiet splash`. Then press either F10 or Ctrl-X to boot (read the text at the bottom of the screen to be sure of the key(s) to press).
- Continue the install as normal
- Shutdown
- Reboot into [Recovery mode](https://wiki.ubuntu.com/RecoveryMode)
  - The secret of booting Ubuntu into recovery mode is to hold the `esc` key and then release it right after the Apple chime is heard.  If the `esc` key is held down for too long, the initial BIO menu will be based and you will go straight to command line GRUB!
  - On some computers/distros, `shift` will initiate recovery mode.)
- Make that `nomodeset` setting permanent: `sudo nano /etc/default/grub`

In that file, add `nomodeset` to `GRUB_CMDLINE_LINUX_DEFAULT` as seen below:
```sh
GRUB_DEFAULT=0
GRUB_HIDDEN_TIMEOUT=0
GRUB_HIDDEN_TIMEOUT_QUIET=true
GRUB_TIMEOUT=5
GRUB_DISTRIBUTOR=`lsb_release -i -s 2> /dev/null || echo Debian`
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash nomodeset"
GRUB_CMDLINE_LINUX=""
```
Save this text file by hitting Ctrl+O, then exit nano with Ctrl+X, then, back in Terminal, run: `sudo update-grub`

- Restart the computer
