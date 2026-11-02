# Troubleshooting

## Intermittent SSH lag

If SSH is intermittently slow, update the ZED Box and disable Wi-Fi power saving:

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo iw dev wlP7p1s0 set power_save off
iw dev wlP7p1s0 get power_save
```

To make the Wi-Fi setting persistent across networks and reboots:

```bash
sudo tee /etc/NetworkManager/conf.d/99-wifi-powersave-off.conf > /dev/null <<'EOF'
[connection]
wifi.powersave=2
EOF
sudo nmcli general reload conf
```
