#!/bin/bash

# Default user/group IDs if not specified
USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}

# Update the group name and GID
groupmod -g $GROUP_ID ubuntu

# Update the user's UID and primary group
usermod -u $USER_ID -g $GROUP_ID ubuntu

# Update home directory name to match new username
cp -f /etc/skel/.bashrc /home/ubuntu/.bashrc

# Add user to common groups for hardware access
usermod -aG sudo,video,audio,plugdev,dialout ubuntu 2>/dev/null || true

# Use gosu to execute command
exec gosu ubuntu "$@"
