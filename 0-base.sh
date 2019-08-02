#!/bin/bash

#Terminal colors
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
blue=`tput setaf 4`
magenta=`tput setaf 5`
cyan=`tput setaf 6`
reset=`tput sgr0`

#################################
### Install Development Tools ###
#################################

echo "${green}Installing Qt Creator${reset}"
sudo apt-get install qt5-default qtcreator -y -q
echo "${red}MANUAL SETUP REQUIRED FOR QT CREATOR BUILD KIT${reset}"

echo "${green}Installing Arduino IDE${reset}"
sudo apt-get install arduino arduino-core -y -q

echo "${green}Installing CMAKE${reset}"
sudo apt-get install cmake -y -q

echo "${green}Installing Git${reset}"
sudo apt-get install git -y -q

echo "${green}Installing Xclip (useful for command line copy of SSH keys)${reset}"
sudo apt-get install xclip -y -q

echo "${green}Installing Meld (Visual Diff Package)${reset}"
sudo apt-get install meld -y -q

echo "${green}Adding exFAT SD Card Support${reset}"
sudo apt-get install exfat-utils exfat-fuse -y -q

echo "${green}Adding LOC Counting Utility${reset}"
sudo apt-get install cloc -y
#https://github.com/AlDanial/cloc

echo "${green}Installing GTK+ and Glade${reset}"
sudo apt-get install libgtk-3-dev -y
sudo apt-get install glade -y

#####################
### Writing Tools ###
#####################

echo "${green}Installing ATOM${reset}"
wget -qO - https://packagecloud.io/AtomEditor/atom/gpgkey | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main" > /etc/apt/sources.list.d/atom.list'
sudo apt-get update
sudo apt-get install atom -y -q

echo "${green}Installing Pandoc${reset}"
sudo apt-get install pandoc pandoc-citeproc -y -q

echo "${green}Installing Poppler-Utils${reset}"
sudo apt install poppler-utils -y
#Gives us pdfunite!!!

echo "${green}Installing Doxygen${reset}"
sudo apt-get install doxygen -y -q

echo "${green}Installing Latext Support${reset}"
sudo apt-get install texlive-xetex -y

############################
### Install System Tools ###
############################

echo "${green}Adding support for FileSRV Drive Mounting${reset}"
sudo apt-get install cifs-utils -y -q

echo "${green}Adding support for SQLITE3${reset}"
sudo apt-get install sqlite3 libsqlite3-dev -y -q

echo "${green}Installing Armadillo${reset}"
sudo apt-get install libarmadillo-dev -y
sudo apt-get install libopenblas-dev -y
sudo apt-get install liblapack-dev -y
sudo apt-get install libarpack-dev -y

echo "${green}Installing System Monitoring Tools${reset}"
#sudo apt-get install indicator-multiload -y -q #For Ubuntu 16 and earlier
#echo "${yellow}Verify that the indicator is set to launch on boot by checking the startup app preferences.  (Search for Startup Applications in the toolbar.)${reset}"
sudo apt install gnome-shell-extension-system-monitor -y #Ubuntu 18+
sudo apt-get install gnome-tweak-tool -y
echo "${yellow}Use the Gnome Tweak Tool to add system monitor and other extensions.${reset}"


###################
### Video Tools ###
###################

echo "${green}Installing FFMPEG${reset}"
sudo apt-get install ffmpeg -y

echo "${green}Installing VLC ${reset}"
sudo apt-get install vlc -y

###################
### Easter Eggs ###
###################

sudo apt-get install lolcat -y
echo "Installing Easter Eggs" | lolcat
sudo apt-get install sl -y
sudo apt-get install fortune-mod -y
sudo apt-get install cowsay -y
fortune | cowsay | lolcat

echo "${cyan}"
cowsay -f tux "Installation Complete"
echo "${reset}"
