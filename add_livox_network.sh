#!/bin/bash

#cleanup 
echo "Removing old livox networks"
sudo nmcli con del livox-master
echo "Adding new"
sudo nmcli con add type ethernet ifname eth0 \
	connection.autoconnect yes \
	con-name livox-master \
	ipv4.method manual \
	ipv4.addresses 192.168.1.1/24 

