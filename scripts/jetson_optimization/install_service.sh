#!/bin/bash

echo "=== Jetson Optimizer Service Installer ==="

# Get current user and workspace path
CURRENT_USER=$(whoami)
WORKSPACE_PATH=$(pwd)

echo "Current user: $CURRENT_USER"
echo "Workspace path: $WORKSPACE_PATH"

# Create service file with correct paths
cat > /tmp/jetson-optimizer.service << EOF
[Unit]
Description=Jetson Orin Performance Optimizer
After=multi-user.target

[Service]
Type=oneshot
ExecStart=$WORKSPACE_PATH/scripts/jetson_optimization/optimize_jetson.sh
User=root
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# Install service
echo "Installing service..."
sudo cp /tmp/jetson-optimizer.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable and start service
echo "Enabling and starting service..."
sudo systemctl enable jetson-optimizer.service
sudo systemctl start jetson-optimizer.service

# Check status
echo "Service status:"
sudo systemctl status jetson-optimizer.service

echo "=== Installation Complete ==="
echo "Service will run automatically on boot."
echo "To check status: sudo systemctl status jetson-optimizer.service"
echo "To view logs: sudo journalctl -u jetson-optimizer.service"
