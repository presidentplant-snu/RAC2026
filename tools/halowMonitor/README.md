# HaLow Connection Monitor

A lightweight Python script to monitor HaLow wireless connection information from an OpenWRT device via SSH.

## Features

- Connects to OpenWRT via SSH
- Retrieves wireless connection metrics:
  - MCS levels (TX/RX)
  - Bandwidth (MHz)
  - RSSI (dBm)
  - Signal strength (dBm)
  - Bitrates (Mbps)
  - Packet counts
  - Channel information
- Client-side processing 
- Single snapshot or continuous monitoring mode

## Installation

1. Install Python 3.6 or higher

2. Install dependencies (or install paramiko via package manager)
```bash
pip install -r requirements.txt
```

## Usage

### Basic usage with password:
```bash
python halow_monitor.py 192.168.1.1 -p your_password
```

### Using SSH key (recommended):
```bash
python halow_monitor.py 192.168.1.1 -k ~/.ssh/id_rsa
```

### Specify interface:
```bash
python halow_monitor.py 192.168.1.1 -k ~/.ssh/id_rsa -i wlan0
```

### Continuous monitoring mode:
```bash
python halow_monitor.py 192.168.1.1 -k ~/.ssh/id_rsa -m
```

### Custom refresh interval (5 seconds):
```bash
python halow_monitor.py 192.168.1.1 -k ~/.ssh/id_rsa -m -r 5
```

## Command Line Options

```bash
python halow_monitor.py host [options]
```
- `host` - IP address or hostname of OpenWRT device (required)

- `-u, --username` - SSH username (default: root)
- `-p, --password` - SSH password
- `-k, --key` - Path to SSH private key file
- `-P, --port` - SSH port (default: 22)
- `-i, --interface` - Wireless interface name (auto-detected if not specified)
- `-m, --monitor` - Enable continuous monitoring mode
- `-r, --refresh` - Refresh interval in seconds for monitor mode (default: 2)
