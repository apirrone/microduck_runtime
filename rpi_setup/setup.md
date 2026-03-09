# Raspberry Pi Zero 2W setup 

## Setup the Pi itself and install the runtime

- Download rpi imager : https://www.raspberrypi.com/software/
- Plug your sd card in your computer
- run it with `sudo`
- Select Raspberry Pi Zero 2 W

<img width="2560" height="718" alt="Capture d’écran du 2026-03-09 11-20-38" src="https://github.com/user-attachments/assets/94ee8d91-677c-41a2-94da-2cc53976c30a" />

- Select Raspberry Pi OS (other)

<img width="2560" height="718" alt="Capture d’écran du 2026-03-09 11-21-15" src="https://github.com/user-attachments/assets/2a9d9338-d87b-4a10-a556-885a57d82240" />

- Select Raspberry Pi OS Lite (64-bit)

<img width="2560" height="718" alt="Capture d’écran du 2026-03-09 11-21-35" src="https://github.com/user-attachments/assets/c3a18dfb-0950-4ddf-82c3-1a09ebf5f64a" />

- Select the sd card

<img width="2560" height="718" alt="Capture d’écran du 2026-03-09 11-23-32" src="https://github.com/user-attachments/assets/ce3a33b7-7182-40e6-a553-afe6716233b1" />

- Name it "microduck"

<img width="659" height="211" alt="Capture d’écran du 2026-03-09 11-24-03" src="https://github.com/user-attachments/assets/d6cab4ad-0418-4ef3-8052-389f90035f87" />

- Select your timezone

<img width="659" height="211" alt="Capture d’écran du 2026-03-09 11-24-31" src="https://github.com/user-attachments/assets/137863c4-942b-43c9-bf62-a09ec1373409" />

- Set microduck as username and set a password (I use just microduck too)

<img width="670" height="256" alt="Capture d’écran du 2026-03-09 11-24-39" src="https://github.com/user-attachments/assets/97fde1b0-f3cf-445c-9fd8-06a410373316" />

- Set the SSID and password of your network (Tip : I use my phone's hotspot so that I can connect anywhere easily)
- 
<img width="685" height="386" alt="Capture d’écran du 2026-03-09 11-25-14" src="https://github.com/user-attachments/assets/de316b0e-176e-43e1-ba04-3b90276896a5" />

- Activate ssh with password authentification

<img width="685" height="386" alt="Capture d’écran du 2026-03-09 11-25-54" src="https://github.com/user-attachments/assets/7b109a03-9b9e-4499-9ff7-fbfe13c47f3f" />

- Write

- Once it's done, put the sd card in your raspi and boot. It should automatically connect to your network.

- Connect your computer to the same network

- You can try to ssh into it with `ssh microduck@microduck.local` but it may not work depending on your router.
- There are two other ways to find the IP of the Pi on the network : 
    - Go to your router's admin interface and look for a raspberry pi in connected devices. You should be able to find the ip address here
    - Find the ip of your computer on the network with `ifconfig`
        - If it's something like `192.168.10.XXX`, run `sudo nmap -sn 192.168.10.0/24`
        - It'll find the devices connected to the network with the same network submask as yours
        
- Connect to the Pi with `ssh microduck@[ip]`
- Tip : in order not to have to enter the password everytime you ssh in the pi, do :
    - login
    - `ssh-keygen` (then press enter until it's done)
    - logout
    - `ssh-copy-id microduck@[ip]`
    - enter password
    - Done ! next time you connect in the pi with `ssh microduck@[ip]` it won't ask for your password
 
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
    - connect [mac address]
    - wait for it to prompt you to accept pairing -> yes
    - trust [mac address]
    
- run `test_controller`

### If issues : 
    
I had this issue : when rebooting, the controller would be stuck in a connect/disconnect loop.

I would have to remove and repair the controller at each reboot.

The solution was to set Privacy to "on" in `/etc/bluetooth/main.conf`. 

Then reboot, remove the device (remove <mac> in bluetoothctl) and do the normal setup above.
