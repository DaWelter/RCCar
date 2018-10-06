#!/bin/sh
cp scripts/rcstation_*.service /etc/systemd/system/ &&
cp scripts/rcstation.target    /etc/systemd/system/ &&
cp scripts/rcstation_*.bash    /usr/local/bin/ &&
cp ../configure_wifi.bash /usr/local/bin/rcstation_configure_wifi.bash &&
systemctl enable rcstation_configure_wifi.service rcstation_udp_rx.service rcstation_udp_rx_video.service rcstation_udp_tx.service
