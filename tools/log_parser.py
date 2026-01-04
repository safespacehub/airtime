#!/usr/bin/env python3
"""
Log Parser for Flight Timer CSV Data
Extracts CSV data from Zephyr log files and writes to a clean CSV file.
"""

import re
import csv
import argparse
import sys
from datetime import datetime


def strip_ansi_codes(text):
    """Remove ANSI escape codes from text."""
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    return ansi_escape.sub('', text)


def parse_timestamp(timestamp_str):
    """Parse Zephyr log timestamp format: [2026-01-04 21:09:31.962,000]"""
    # Remove brackets
    timestamp_str = timestamp_str.strip('[]')
    # Parse the timestamp
    try:
        # Format: 2026-01-04 21:09:31.962,000
        dt = datetime.strptime(timestamp_str.split(',')[0], '%Y-%m-%d %H:%M:%S.%f')
        return dt.isoformat()
    except ValueError:
        return timestamp_str


def extract_csv_from_log(log_file_path, output_file_path):
    """
    Extract CSV data from Zephyr log file.
    
    Log format: [timestamp] [0m<inf> flight_timer: CSV,baro_state,gps_state,lat,lon,speed,alt,temp,p1,p2,p3...[0m
    """
    csv_lines = []
    
    # Pattern to match log lines with CSV data
    # Format: [timestamp] ... flight_timer: CSV,...
    csv_pattern = re.compile(r'\[([^\]]+)\].*?flight_timer:\s+CSV,(.+)')
    
    with open(log_file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line_num, line in enumerate(f, 1):
            # Strip ANSI codes
            clean_line = strip_ansi_codes(line)
            
            # Try to match CSV pattern
            match = csv_pattern.search(clean_line)
            if match:
                timestamp_str = match.group(1)
                csv_data = match.group(2).strip()
                
                # Remove trailing ANSI codes that might remain
                csv_data = strip_ansi_codes(csv_data).rstrip()
                
                # Parse timestamp
                timestamp = parse_timestamp(timestamp_str)
                
                # Parse CSV fields
                fields = csv_data.split(',')
                if len(fields) >= 7:  # Minimum: baro_state, gps_state, lat, lon, speed, alt, temp
                    csv_lines.append({
                        'timestamp': timestamp,
                        'raw_fields': fields
                    })
    
    if not csv_lines:
        print(f"Warning: No CSV data found in {log_file_path}", file=sys.stderr)
        return 1
    
    # Write to CSV file
    with open(output_file_path, 'w', newline='') as csvfile:
        # Build header - single pressure column
        header = [
            'timestamp',
            'baro_state',
            'gps_state',
            'latitude',
            'longitude',
            'speed_kts',
            'altitude_m',
            'temperature_c',
            'pressure'
        ]
        
        writer = csv.DictWriter(csvfile, fieldnames=header)
        writer.writeheader()
        
        # Write data rows - expand each log line into multiple rows (one per pressure sample)
        total_rows = 0
        for entry in csv_lines:
            fields = entry['raw_fields']
            
            # Base row data (same for all pressure samples from this log line)
            base_row = {
                'timestamp': entry['timestamp'],
                'baro_state': fields[0] if len(fields) > 0 else '',
                'gps_state': fields[1] if len(fields) > 1 else '',
                'latitude': fields[2] if len(fields) > 2 else '',
                'longitude': fields[3] if len(fields) > 3 else '',
                'speed_kts': fields[4] if len(fields) > 4 else '',
                'altitude_m': fields[5] if len(fields) > 5 else '',
                'temperature_c': fields[6] if len(fields) > 6 else '',
            }
            
            # Extract pressure samples (fields 7 onwards)
            # Fields: baro_state, gps_state, lat, lon, speed, alt, temp, then pressure samples
            pressure_start_idx = 7
            pressure_samples = []
            
            for i in range(pressure_start_idx, len(fields)):
                pressure_val = fields[i].strip()
                if pressure_val:  # Only add non-empty pressure values
                    pressure_samples.append(pressure_val)
            
            # Create one row per pressure sample
            if pressure_samples:
                for pressure in pressure_samples:
                    row = base_row.copy()
                    row['pressure'] = pressure
                    writer.writerow(row)
                    total_rows += 1
            else:
                # If no pressure samples, write one row with empty pressure
                row = base_row.copy()
                row['pressure'] = ''
                writer.writerow(row)
                total_rows += 1
    
    print(f"Extracted {len(csv_lines)} log entries from {log_file_path}")
    print(f"Generated {total_rows} CSV rows (expanded by pressure samples)")
    print(f"Output written to {output_file_path}")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Extract CSV data from Zephyr flight timer log files'
    )
    parser.add_argument(
        'log_file',
        help='Path to the log file to parse'
    )
    parser.add_argument(
        '-o', '--output',
        help='Output CSV file path (default: flight_data.csv)',
        default='flight_data.csv'
    )
    
    args = parser.parse_args()
    
    try:
        return extract_csv_from_log(args.log_file, args.output)
    except FileNotFoundError:
        print(f"Error: Log file not found: {args.log_file}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

