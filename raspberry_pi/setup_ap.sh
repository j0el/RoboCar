#!/usr/bin/env bash
# setup_ap.sh — turn the Pi into a WiFi access point for the phone controller.
#
# Creates a NetworkManager hotspot profile (Raspberry Pi OS Bookworm uses
# NetworkManager) on wlan0:
#     SSID:     RoboCar
#     Password: robocar1  (WPA2)
#     Pi IP:    10.42.0.1 (NetworkManager "shared" mode default; the phone
#               gets an address via the built-in DHCP)
#
# Run ONCE on the Pi:   sudo bash setup_ap.sh
# The profile autoconnects on boot from then on.
#
# NOTE: while the hotspot is up, wlan0 has no internet. To get the Pi back
# online (e.g. to git pull), plug in ethernet, or switch profiles:
#     sudo nmcli connection down RoboCarAP     # stop the hotspot
#     sudo nmcli connection up   RoboCarAP     # start it again
#
# The phone connects to WiFi "RoboCar", then opens http://10.42.0.1:8080
# (the cone_visitor.py web UI / Android app endpoint).

set -euo pipefail

SSID="RoboCar"
PSK="robocar1"
CON="RoboCarAP"
IFACE="wlan0"

if ! command -v nmcli >/dev/null; then
    echo "nmcli not found — this script needs Raspberry Pi OS Bookworm (NetworkManager)." >&2
    exit 1
fi

# Make sure WiFi is enabled and not soft-blocked (fresh installs without a
# configured country can rfkill-block the radio)
rfkill unblock wifi 2>/dev/null || true
nmcli radio wifi on

# Recreate the profile from scratch so re-running the script is safe
nmcli connection delete "$CON" 2>/dev/null || true

nmcli connection add type wifi ifname "$IFACE" con-name "$CON" autoconnect yes \
    ssid "$SSID" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    ipv4.method shared \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PSK"

nmcli connection up "$CON"

echo
echo "Hotspot '$SSID' is up (password: $PSK)."
echo "Connect the phone to it and open http://10.42.0.1:8080"
