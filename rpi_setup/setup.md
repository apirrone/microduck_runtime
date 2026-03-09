# Raspberry Pi Zero 2W setup 

## Setup the Pi itself and install the runtime

- Download rpi imager : https://www.raspberrypi.com/software/
- Plug your sd card in your computer
- run it with `sudo`
- Select Raspberry Pi Zero 2 W 

<image>

- Select Raspberry Pi OS (other)

<image>

- Select Raspberry Pi OS Lite (64-bit)

<image>

- Select the sd card

<image>

- Name it "microduck"

<image>

- Select your timezone

<image>

- Set microduck as username and set a password (I use just microduck too)

<image>

- Set the SSID and password of your network (Tip : I use my phone's hotspot so that I can connect anywhere easily)

<image>

- Activate ssh with password

<image>

- Write

- Once it's done, put the sd card in your raspi and boot. It should automatically connect to your network.

- Connect your computer to the same network

- You can try to ssh into it with `ssh microduck@microduck.local` but it may not work depending on your router.
- There are two other ways to find the IP of the Pi on the network : 
    - Go to your router's admin interface and look for a raspberry pi in connected devices. You should be able to find the ip address here
    - Find the ip of your computer on the network with `ifconfig`
        - If it's something like `192.168.10.XXX`, run `sudo nmap -sn 192.168.10.0/24`
        - It'll find the devices connected to the network with the same network submask as yours
        
- Connect to the Pi with `ssh microduck@<ip>
- Tip : in order not to have to enter the password everytime you ssh in the pi, do :
    - login
    - ssh-keygen (then press enter until it's done)
    - logout
    - ssh-copy-id microduck@<ip>
    - enter password
    - Done ! next time you connect in the pi with `ssh microduck@<ip>` it won't ask for your password
 
- `scp rpi_setup/config.txt in /boot/firmware/`
- run `sudo raspi-config`
    - interface
        - enable i2c
        - disable serial console, enable serial port
- `sudo reboot 0`
- `curl -sSL https://raw.githubusercontent.com/apirrone/microduck_runtime/main/install.sh | bash`

## Xbox controller

### Normal setup :
- `bluetoothctl` : 
    - scan on (and long press pairing button on the controller)
    - connect <mac address>
    - wait for it to prompt you to accept pairing -> yes
    - trust <mac address>
    
- run `test_controller`

### If issues : 
    
I had this issue : when rebooting, the controller would be stuck in a connect/disconnect loop.

I would have to remove and repair the controller at each reboot.

The solution was to set Privacy to "on" in `/etc/bluetooth/main.conf`. 

Then reboot, remove the device (remove <mac> in bluetoothctl) and do the normal setup above.