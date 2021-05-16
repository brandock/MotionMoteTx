#!/bin/sh

# Enable and disable HDMI output on the Raspberry Pi

# Make it executable with chmod +x rpi-hdmi.sh

# Also, if you get an ^M error it is because FileZilla transmits bad characters that can't be used. Cut and paste instead.

is_off ()
{
        vcgencmd display_power | grep "display_power=0" >/dev/null
}

case $1 in
        0)
                if is_off
                then
                        echo Already off...
                else
                        vcgencmd display_power 0
                fi
        ;;
        1)
                if is_off
                then
                        vcgencmd display_power 1
                else
                        echo Already on...
                fi
        ;;
        status)
                if is_off
                then
                        echo off
                else
                        echo on
                fi
        ;;
        *)
                echo "Usage: $0 1|0|status" >&2
                exit 2
        ;;
esac

exit 0

