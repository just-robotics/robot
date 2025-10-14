#!/bin/bash
# Script to generate wpa_supplicant.conf and static network config using systemd-networkd
# Sets Wi-Fi country to Russia and static IP based on robot index
# Must be run as root

# ===== Settings =====
SSID="helldivers"
GATEWAY="192.168.3.1"
DNS="8.8.8.8 8.8.4.4"
BASE_NET="192.168.3"
ETH_NET="10.10.10"
# ====================

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <ROOTFS_PATH> <ROBOT_INDEX>"
    exit 1
fi

ROOTFS="$1"
INDEX="$2"
ROOTFS="${ROOTFS%/}"

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo $0 $ROOTFS $INDEX"
    exit 1
fi

if [ ! -d "$ROOTFS" ]; then
    echo "Error: Rootfs path '$ROOTFS' does not exist."
    exit 1
fi

LAST_OCTET=$((100 + INDEX))
if [ "$LAST_OCTET" -gt 254 ]; then
    echo "Error: calculated IP last octet exceeds 254"
    exit 1
fi
STATIC_IP_WIFI="$BASE_NET.$LAST_OCTET"
STATIC_IP_ETH="$ETH_NET.$LAST_OCTET"

read -s -p "Enter Wi-Fi password for $SSID: " PASSWORD
echo

# wpa_supplicant
mkdir -p "$ROOTFS/etc/wpa_supplicant"
cat > "$ROOTFS/etc/wpa_supplicant/wpa_supplicant.conf" <<EOF
ctrl_interface=DIR=/run/wpa_supplicant GROUP=netdev
update_config=1
country=RU
network={
    ssid="$SSID"
    psk=$(wpa_passphrase "$SSID" "$PASSWORD" | grep '^\s*psk=' | cut -d= -f2)
}
EOF
chmod 600 "$ROOTFS/etc/wpa_supplicant/wpa_supplicant.conf"

# systemd-networkd config
mkdir -p "$ROOTFS/etc/systemd/network"

# Wi-Fi
cat > "$ROOTFS/etc/systemd/network/20-wlan0.network" <<EOF
[Match]
Name=wlan0

[Network]
Address=$STATIC_IP_WIFI/24
Gateway=$GATEWAY
DNS=$DNS
EOF

# Ethernet
cat > "$ROOTFS/etc/systemd/network/30-eth0.network" <<EOF
[Match]
Name=eth0

[Network]
Address=$STATIC_IP_ETH/24
Gateway=$ETH_NET.1
DNS=$DNS
EOF

# Enable systemd-networkd and wpa_supplicant
mkdir -p "$ROOTFS/etc/systemd/system/multi-user.target.wants"
ln -sf /lib/systemd/system/systemd-networkd.service "$ROOTFS/etc/systemd/system/multi-user.target.wants/systemd-networkd.service"
ln -sf /lib/systemd/system/wpa_supplicant.service "$ROOTFS/etc/systemd/system/multi-user.target.wants/wpa_supplicant.service"

# Unblock Wi-Fi on first boot
cat > "$ROOTFS/etc/systemd/system/unblock-wifi.service" <<EOF
[Unit]
Description=Unblock Wi-Fi
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'rfkill unblock wifi'
ExecStart=/bin/bash -c 'systemctl disable NetworkManager'
ExecStart=/bin/bash -c 'wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf'
ExecStart=/bin/bash -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'
ExecStart=/bin/bash -c 'echo "nameserver 8.8.4.4" >> /etc/resolv.conf'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$ROOTFS/etc/systemd/system/unblock-wifi.service"
ln -sf /etc/systemd/system/unblock-wifi.service "$ROOTFS/etc/systemd/system/multi-user.target.wants/unblock-wifi.service"

# Add alias for robot setup in .bashrc =====
BASHRC="$ROOTFS/home/pi/.bashrc"

HELPER_SCRIPT="robot_setup.sh"
cp "$(dirname "$0")/$HELPER_SCRIPT" "$ROOTFS/home/pi/.$HELPER_SCRIPT"
chmod +x "$ROOTFS/home/pi/.$HELPER_SCRIPT"

# Add alias to .bashrc if not already present
BASHRC="$ROOTFS/home/pi/.bashrc"
grep -qxF "alias robot_setup='/home/pi/.$HELPER_SCRIPT'" "$BASHRC" || \
echo "alias robot_setup='/home/pi/.$HELPER_SCRIPT'" >> "$BASHRC"
grep -qxF "rfkill unblock all" "$BASHRC" || echo "rfkill unblock all" >> "$BASHRC"
grep -qxF "ROBOT_ID=$INDEX" "$BASHRC" || echo "ROBOT_ID=$INDEX" >> "$BASHRC"
# =================================================

sync

echo "Systemd-networkd and wpa_supplicant configs created."
echo "Wi-Fi: $STATIC_IP_WIFI"
echo "Ethernet: $STATIC_IP_ETH"
