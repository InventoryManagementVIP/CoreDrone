#!/usr/bin/env bash
# One-time setup of a CoreDrone Jetson (JetPack 7.2). UNTESTED SKELETON: no Jetson yet.
# Usage: sudo bash real/jetson/setup.sh   (from a checkout at /opt/coredrone)
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }

echo "== JetPack check"
if ! grep -q "R39" /etc/nv_tegra_release 2>/dev/null; then
  echo "warning: expected Jetson Linux R39 (JetPack 7.2); see docs/adr/0002-ros-distro.md"
fi

echo "== Docker + NVIDIA container runtime"
command -v docker >/dev/null || apt-get install -y docker.io docker-compose-v2
docker info 2>/dev/null | grep -qi nvidia || echo "warning: NVIDIA container runtime not configured (nvidia-ctk runtime configure --runtime=docker)"

echo "== chrony (time sync)"
apt-get install -y chrony
install -m 0644 "$REPO/real/jetson/chrony.conf" /etc/chrony/conf.d/coredrone.conf
systemctl restart chrony

echo "== udev rules (camera, flight controller)"
install -m 0644 "$REPO/real/sensors/udev/99-coredrone.rules" /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger

echo "== mavlink-router (FC telemetry -> GCS and local tools)"
if ! command -v mavlink-routerd >/dev/null; then
  echo "install mavlink-router from https://github.com/mavlink-router/mavlink-router (build from source)"
fi
install -d /etc/mavlink-router
install -m 0644 "$REPO/real/jetson/mavlink-router.conf" /etc/mavlink-router/main.conf

echo "== environment file (secrets stay out of git)"
install -d -m 0750 /etc/coredrone
[ -f /etc/coredrone/coredrone.env ] || install -m 0600 "$REPO/.env.example" /etc/coredrone/coredrone.env
echo "   edit /etc/coredrone/coredrone.env (ROS_DOMAIN_ID, backend URL/key)"

echo "== systemd unit"
install -m 0644 "$REPO/real/jetson/systemd/coredrone.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable coredrone.service
echo "done. start with: systemctl start coredrone"
