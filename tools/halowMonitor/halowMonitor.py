#!/usr/bin/env python3
"""
Morse Micro AP Link Monitor
Monitors link statistics via SSH and displays real-time metrics
"""

import argparse
import json
import re
import sys
import time
from datetime import timedelta
from typing import Dict, Optional

try:
    import paramiko
except ImportError:
    print("Error: paramiko module not found. Install with: pip install paramiko")
    sys.exit(1)


class Colors:
    """ANSI color codes"""
    RED = '\033[91m'
    YELLOW = '\033[93m'
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'
    
    @classmethod
    def disable(cls):
        """Disable all colors"""
        cls.RED = cls.YELLOW = cls.GREEN = cls.BLUE = cls.CYAN = cls.BOLD = cls.RESET = ''


class MorseMonitor:
    def __init__(self, host: str, user: str, password: str, port: int, interface: str, use_color: bool):
        self.host = host
        self.user = user
        self.password = password
        self.port = port
        self.interface = interface
        self.use_color = use_color
        self.ssh = None
        self.cycle_count = 0
        self.cached_stats = {}
        
        if not use_color:
            Colors.disable()
    
    def connect(self):
        """Establish SSH connection"""
        try:
            self.ssh = paramiko.SSHClient()
            self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh.connect(
                hostname=self.host,
                port=self.port,
                username=self.user,
                password=self.password,
                timeout=5
            )
            print(f"{Colors.GREEN}✓ Connected to {self.host}{Colors.RESET}")
        except Exception as e:
            print(f"{Colors.RED}✗ Failed to connect: {e}{Colors.RESET}")
            sys.exit(1)
    
    def disconnect(self):
        """Close SSH connection"""
        if self.ssh:
            self.ssh.close()
    
    def exec_command(self, cmd: str) -> str:
        """Execute SSH command and return output"""
        try:
            stdin, stdout, stderr = self.ssh.exec_command(cmd, timeout=5)
            return stdout.read().decode('utf-8')
        except Exception as e:
            return f"Error: {e}"
    
    def get_iw_stats(self) -> Optional[Dict]:
        """Parse iw station dump output"""
        output = self.exec_command(f"iw dev {self.interface} station dump")
        if "Error" in output or not output.strip():
            return None
        
        stats = {}
        for line in output.split('\n'):
            line = line.strip()
            
            if line.startswith('Station'):
                stats['mac'] = line.split()[1]
            elif 'inactive time:' in line:
                stats['inactive_time'] = line.split(':')[1].strip()
            elif 'signal:' in line and 'signal avg:' not in line:
                match = re.search(r'(-?\d+)\s*dBm', line)
                if match:
                    stats['signal'] = int(match.group(1))
            elif 'signal avg:' in line:
                match = re.search(r'(-?\d+)\s*dBm', line)
                if match:
                    stats['signal_avg'] = int(match.group(1))
            elif 'tx bitrate:' in line:
                stats['tx_bitrate'] = line.split(':', 1)[1].strip()
            elif 'rx bitrate:' in line:
                stats['rx_bitrate'] = line.split(':', 1)[1].strip()
            elif 'expected throughput:' in line:
                stats['expected_throughput'] = line.split(':')[1].strip()
            elif 'tx retries:' in line:
                stats['tx_retries'] = int(line.split(':')[1].strip())
            elif 'tx failed:' in line:
                stats['tx_failed'] = int(line.split(':')[1].strip())
            elif 'connected time:' in line:
                stats['connected_time'] = int(line.split(':')[1].split()[0].strip())
        
        return stats if stats else None
    
    def get_morse_stats(self) -> Dict:
        """Parse morse_cli stats output with optimized polling"""
        stats = {}
        
        # Poll temperature and uptime every 5 cycles (5 seconds) - these don't change quickly
        if self.cycle_count % 5 == 0:
            # Combine Apps and PHY stats in one command to reduce SSH overhead
            combined_output = self.exec_command("morse_cli stats -aj && echo '---SEPARATOR---' && morse_cli stats -uj")
            
            try:
                parts = combined_output.split('---SEPARATOR---')
                
                # Apps stats
                if len(parts) > 0:
                    apps_data = json.loads(parts[0].strip())
                    self.cached_stats['uptime'] = apps_data.get('System uptime (us)', 0)
                    self.cached_stats['apps_cpu'] = apps_data.get('apps cpu utilisation (tenths of a percent)', 0)
                
                # PHY stats
                if len(parts) > 1:
                    phy_data = json.loads(parts[1].strip())
                    self.cached_stats['temperature'] = phy_data.get('Temperature(Celcius)', 0)
                    self.cached_stats['noise'] = phy_data.get('Noise dBm', 0)
                    self.cached_stats['phy_cpu'] = phy_data.get('phy cpu utilisation (tenths of a percent)', 0)
            except Exception as e:
                pass
        
        # Always poll MAC stats (these change frequently with traffic)
        mac_output = self.exec_command("morse_cli stats -mj")
        try:
            mac_data = json.loads(mac_output)
            stats['tx_success_pct'] = mac_data.get('TX round-trip success %', 0)
            stats['tx_ack_timeout'] = mac_data.get('TX ACK Timeout', 0)
            stats['rx_total'] = mac_data.get('RX Total', 0)
            stats['rx_pass_fcs'] = mac_data.get('RX Pass FCS', 0)
            stats['ampdu_tx'] = mac_data.get('AGG N aggregates total', 0)
            stats['ampdu_rx'] = mac_data.get('RX A-MPDU/S-MPDU', 0)
            stats['mac_cpu'] = mac_data.get('mac cpu utilisation (tenths of a percent)', 0)
        except:
            pass
        
        # Merge cached stats
        stats.update(self.cached_stats)
        
        return stats
    
    def format_uptime(self, uptime_us: int) -> str:
        """Convert microseconds to human readable format"""
        seconds = uptime_us / 1_000_000
        return str(timedelta(seconds=int(seconds)))
    
    def format_connected_time(self, seconds: int) -> str:
        """Format connected time"""
        return str(timedelta(seconds=seconds))
    
    def colorize_value(self, value: float, thresholds: Dict, fmt: str = '.0f') -> str:
        """Apply color based on thresholds and format the value"""
        formatted = format(value, fmt)
        
        if not self.use_color:
            return formatted
        
        if value >= thresholds.get('green', float('inf')):
            return f"{Colors.GREEN}{formatted}{Colors.RESET}"
        elif value >= thresholds.get('yellow', float('inf')):
            return f"{Colors.YELLOW}{formatted}{Colors.RESET}"
        else:
            return f"{Colors.RED}{formatted}{Colors.RESET}"
    
    def display_stats(self, iw_stats: Dict, morse_stats: Dict):
        """Display formatted statistics"""
        # Clear screen and move cursor to top
        print("\033[2J\033[H", end='')
        
        print(f"{Colors.BOLD}{'='*80}{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}Morse Micro AP Monitor - {self.interface} @ {self.host}{Colors.RESET}")
        print(f"{Colors.BOLD}{'='*80}{Colors.RESET}\n")
        
        # Link Quality Section
        print(f"{Colors.BOLD}{Colors.BLUE}Link Quality:{Colors.RESET}")
        
        if iw_stats:
            # TX info
            tx_bitrate = iw_stats.get('tx_bitrate', 'N/A')
            print(f"  TX: {Colors.BOLD}{tx_bitrate}{Colors.RESET}")
            
            # RX info
            rx_bitrate = iw_stats.get('rx_bitrate', 'N/A')
            expected_tp = iw_stats.get('expected_throughput', 'N/A')
            print(f"  RX: {Colors.BOLD}{rx_bitrate}{Colors.RESET} | Expected: {expected_tp}")
            
            # Signal and Noise
            signal = iw_stats.get('signal', 0)
            signal_avg = iw_stats.get('signal_avg', 0)
            noise = morse_stats.get('noise', 0)
            
            # Calculate SNR
            snr = signal - noise if signal and noise else 0
            
            # Color code signal strength
            signal_color = Colors.GREEN if signal > -50 else (Colors.YELLOW if signal > -70 else Colors.RED)
            snr_color = Colors.GREEN if snr > 40 else (Colors.YELLOW if snr > 20 else Colors.RED)
            
            print(f"  Signal: {signal_color}{signal} dBm{Colors.RESET} (avg: {signal_avg} dBm) | "
                  f"Noise: {noise} dBm | SNR: {snr_color}{snr} dB{Colors.RESET}")
        else:
            print(f"  {Colors.RED}No station data available{Colors.RESET}")
        
        print()
        
        # Performance Section
        print(f"{Colors.BOLD}{Colors.BLUE}Performance:{Colors.RESET}")
        
        tx_success = morse_stats.get('tx_success_pct', 0)
        tx_retries = iw_stats.get('tx_retries', 0) if iw_stats else 0
        tx_failed = iw_stats.get('tx_failed', 0) if iw_stats else 0
        tx_ack_timeout = morse_stats.get('tx_ack_timeout', 0)
        
        # Color code TX success
        success_str = self.colorize_value(tx_success, {'green': 95, 'yellow': 90}, '.0f')
        
        print(f"  TX Success: {success_str}% | Retries: {tx_retries} | "
              f"Failed: {tx_failed} | ACK Timeout: {tx_ack_timeout}")
        
        # RX FCS stats
        rx_total = morse_stats.get('rx_total', 0)
        rx_pass = morse_stats.get('rx_pass_fcs', 0)
        rx_pass_pct = (rx_pass / rx_total * 100) if rx_total > 0 else 0
        
        fcs_str = self.colorize_value(rx_pass_pct, {'green': 98, 'yellow': 95}, '.1f')
        
        print(f"  RX: {rx_pass}/{rx_total} FCS pass ({fcs_str}%)")
        
        # A-MPDU stats
        ampdu_tx = morse_stats.get('ampdu_tx', 0)
        ampdu_rx = morse_stats.get('ampdu_rx', 0)
        print(f"  A-MPDU: TX {ampdu_tx} aggregates | RX {ampdu_rx} aggregates")
        
        print()
        
        # System Section
        print(f"{Colors.BOLD}{Colors.BLUE}System:{Colors.RESET}")
        
        temp = morse_stats.get('temperature', 0)
        temp_color = Colors.GREEN if temp < 60 else (Colors.YELLOW if temp < 75 else Colors.RED)
        
        apps_cpu = morse_stats.get('apps_cpu', 0) / 10.0
        mac_cpu = morse_stats.get('mac_cpu', 0) / 10.0
        phy_cpu = morse_stats.get('phy_cpu', 0) / 10.0
        
        print(f"  Temp: {temp_color}{temp}°C{Colors.RESET} | "
              f"CPU: Apps {apps_cpu:.1f}% | MAC {mac_cpu:.1f}% | PHY {phy_cpu:.1f}%")
        
        uptime = morse_stats.get('uptime', 0)
        uptime_str = self.format_uptime(uptime)
        
        connected_time = iw_stats.get('connected_time', 0) if iw_stats else 0
        connected_str = self.format_connected_time(connected_time)
        
        print(f"  Uptime: {uptime_str} | Connected: {connected_str}")
        
        print(f"\n{Colors.BOLD}{'='*80}{Colors.RESET}")
        print(f"Press Ctrl+C to exit")
        
        # Increment cycle counter
        self.cycle_count += 1
    
    def run(self, interval: float):
        """Main monitoring loop"""
        self.connect()
        
        try:
            while True:
                iw_stats = self.get_iw_stats()
                morse_stats = self.get_morse_stats()
                self.display_stats(iw_stats, morse_stats)
                time.sleep(interval)
        
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}Monitoring stopped{Colors.RESET}")
        finally:
            self.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description='Monitor Morse Micro AP link statistics in real-time',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument('-i', '--interval', type=float, default=1.0,
                        help='Update interval in seconds')
    parser.add_argument('-H', '--host', default='192.168.149.1',
                        help='SSH hostname or IP address')
    parser.add_argument('-u', '--user', default='root',
                        help='SSH username')
    parser.add_argument('-P', '--password', default='heltec.org',
                        help='SSH password')
    parser.add_argument('-p', '--port', type=int, default=22,
                        help='SSH port')
    parser.add_argument('--interface', default='wlan0.sta1',
                        help='Wireless interface name')
    parser.add_argument('--no-color', action='store_true',
                        help='Disable color output')
    
    args = parser.parse_args()
    
    monitor = MorseMonitor(
        host=args.host,
        user=args.user,
        password=args.password,
        port=args.port,
        interface=args.interface,
        use_color=not args.no_color
    )
    
    monitor.run(args.interval)


if __name__ == '__main__':
    main()
