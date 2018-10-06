#!/bin/sh
cp scripts/rccar.target /etc/systemd/system/ && \
cp scripts/rccar_*.service /etc/systemd/system/ && \
cp scripts/rccar_*.bash /usr/local/bin/ && \
cp ../configure_wifi.bash /usr/local/bin/rccar_configure_wifi.bash && \
cp rccar_shutdownbutton.py /usr/local/bin/ && \
cp rccar_vehiclestatemonitor.py /usr/local/bin/ && \
cp rccar_control_and_reporting_script.py /usr/local/bin

# After install, don't forget to enable all the services:
# systemctl enable rccar_video_tx.service .. and so on.
# The services will activate when the rccar.target is systemctl isolate'd.
