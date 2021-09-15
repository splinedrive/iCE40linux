just connect your icebreaker to usb!

type:

./flash_system.sh

open a terminal

minicom -D /dev/ttyUSBx -b 1000000

you can write on ubi filessystem, when you remount the /
via mount -o remount,rw /

Have fun
Hirosh

