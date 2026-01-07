#!/usr/bin/env python3
"""
Log Parser for Flight Timer CSV Data
Extracts CSV data from Zephyr log files and writes to a clean CSV file.
Supports multiple input files and segment detection based on timestamp gaps.
"""

import re
import csv
import argparse
import sys
from datetime import datetime, timedelta
from typing import List, Dict, Any


def strip_ansi_codes(text):
    """Remove ANSI escape codes from text."""
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    return ansi_escape.sub('', text)


def parse_timestamp(timestamp_str):
    """
    Parse Zephyr log timestamp format: [2026-01-04 21:09:31.962,000]
    Returns datetime object or None if parsing fails.
    """
    # Remove brackets
    timestamp_str = timestamp_str.strip('[]')
    # Parse the timestamp
    try:
        # Format: 2026-01-04 21:09:31.962,000
        dt = datetime.strptime(timestamp_str.split(',')[0], '%Y-%m-%d %H:%M:%S.%f')
        return dt
    except ValueError:
        return None


def extract_csv_from_logs(log_file_paths: List[str]) -> List[Dict[str, Any]]:
    """
    Extract CSV data from multiple Zephyr log files.
    
    Log format: [timestamp] [0m<inf> flight_timer: CSV,flight_status,baro_state,gps_state,lat,lon,speed,alt,temp,p1,p2,p3...[0m
    
    Returns a list of entries with 'timestamp' (datetime) and 'raw_fields' (list).
    """
    all_csv_lines = []
    
    # Pattern to match log lines with CSV data
    # Format: [timestamp] ... flight_timer: CSV,...
    csv_pattern = re.compile(r'\[([^\]]+)\].*?flight_timer:\s+CSV,(.+)')
    
    for log_file_path in log_file_paths:
        csv_lines = []
        
        try:
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
                        if timestamp is None:
                            continue  # Skip entries with invalid timestamps
                        
                        # Parse CSV fields
                        fields = csv_data.split(',')
                        if len(fields) >= 8:  # Minimum: flight_status, baro_state, gps_state, lat, lon, speed, alt, temp
                            csv_lines.append({
                                'timestamp': timestamp,
                                'raw_fields': fields,
                                'source_file': log_file_path
                            })
        except FileNotFoundError:
            print(f"Warning: Log file not found: {log_file_path}", file=sys.stderr)
            continue
        except Exception as e:
            print(f"Warning: Error reading {log_file_path}: {e}", file=sys.stderr)
            continue
        
        all_csv_lines.extend(csv_lines)
        print(f"Extracted {len(csv_lines)} log entries from {log_file_path}")
    
    if not all_csv_lines:
        print("Error: No CSV data found in any log file", file=sys.stderr)
        return []
    
    # Sort by timestamp
    all_csv_lines.sort(key=lambda x: x['timestamp'])
    
    return all_csv_lines


def split_into_segments(csv_lines: List[Dict[str, Any]], gap_minutes: int = 10) -> List[List[Dict[str, Any]]]:
    """
    Split CSV lines into segments based on timestamp gaps.
    
    Args:
        csv_lines: List of CSV entries sorted by timestamp
        gap_minutes: Maximum gap in minutes before starting a new segment (default: 10)
    
    Returns:
        List of segments, where each segment is a list of CSV entries
    """
    if not csv_lines:
        return []
    
    segments = []
    current_segment = [csv_lines[0]]
    
    for i in range(1, len(csv_lines)):
        time_diff = csv_lines[i]['timestamp'] - csv_lines[i-1]['timestamp']
        
        # If gap is more than gap_minutes, start a new segment
        if time_diff > timedelta(minutes=gap_minutes):
            segments.append(current_segment)
            current_segment = [csv_lines[i]]
        else:
            current_segment.append(csv_lines[i])
    
    # Add the last segment
    if current_segment:
        segments.append(current_segment)
    
    return segments


def format_segment_info(segment: List[Dict[str, Any]], index: int) -> str:
    """Format segment information for display."""
    if not segment:
        return f"Segment {index + 1}: (empty)"
    
    start_time = segment[0]['timestamp']
    end_time = segment[-1]['timestamp']
    duration = end_time - start_time
    
    # Count total rows (including pressure samples)
    total_rows = 0
    for entry in segment:
        fields = entry['raw_fields']
        pressure_start_idx = 8
        pressure_count = sum(1 for i in range(pressure_start_idx, len(fields)) if fields[i].strip())
        total_rows += pressure_count if pressure_count > 0 else 1
    
    return (f"Segment {index + 1}: "
            f"{start_time.strftime('%Y-%m-%d %H:%M:%S')} to "
            f"{end_time.strftime('%Y-%m-%d %H:%M:%S')} "
            f"({duration.total_seconds() / 60:.1f} min, {len(segment)} entries, {total_rows} rows)")


def export_segment_to_csv(segment: List[Dict[str, Any]], output_file_path: str) -> int:
    """Export a segment to CSV file."""
    if not segment:
        print("Error: Segment is empty", file=sys.stderr)
        return 1
    
    # Write to CSV file
    with open(output_file_path, 'w', newline='') as csvfile:
        # Build header - single pressure column
        header = [
            'timestamp',
            'flight_status',
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
        for entry_idx, entry in enumerate(segment):
            fields = entry['raw_fields']
            
            # Base row data (same for all pressure samples from this log line)
            # Fields: flight_status, baro_state, gps_state, lat, lon, speed, alt, temp, then pressure samples
            base_row = {
                'flight_status': fields[0] if len(fields) > 0 else '',
                'baro_state': fields[1] if len(fields) > 1 else '',
                'gps_state': fields[2] if len(fields) > 2 else '',
                'latitude': fields[3] if len(fields) > 3 else '',
                'longitude': fields[4] if len(fields) > 4 else '',
                'speed_kts': fields[5] if len(fields) > 5 else '',
                'altitude_m': fields[6] if len(fields) > 6 else '',
                'temperature_c': fields[7] if len(fields) > 7 else '',
            }
            
            # Extract pressure samples (fields 8 onwards)
            pressure_start_idx = 8
            pressure_samples = []
            
            for i in range(pressure_start_idx, len(fields)):
                pressure_val = fields[i].strip()
                if pressure_val:  # Only add non-empty pressure values
                    pressure_samples.append(pressure_val)
            
            # Calculate timestamp interpolation
            # Samples are collected at 10 Hz (every 100ms) and logged once per second
            # Interpolate timestamps evenly across the 1-second interval
            current_timestamp = entry['timestamp']
            
            # Determine the time span for interpolation
            # If there's a next entry, use the time difference; otherwise assume 1 second
            if entry_idx < len(segment) - 1:
                next_timestamp = segment[entry_idx + 1]['timestamp']
                time_span = (next_timestamp - current_timestamp).total_seconds()
            else:
                # Last entry: assume 1 second interval (standard logging interval)
                time_span = 1.0
            
            # Create one row per pressure sample with interpolated timestamps
            if pressure_samples:
                num_samples = len(pressure_samples)
                # Distribute samples evenly across the time span
                # First sample at current timestamp, last sample just before next timestamp
                for sample_idx, pressure in enumerate(pressure_samples):
                    row = base_row.copy()
                    # Interpolate timestamp: distribute evenly across time_span
                    if num_samples > 1:
                        # Fraction of time span for this sample (0.0 to 1.0)
                        fraction = sample_idx / (num_samples - 1)
                        # Calculate offset in seconds
                        offset_seconds = fraction * time_span
                        # Create interpolated timestamp
                        interpolated_timestamp = current_timestamp + timedelta(seconds=offset_seconds)
                        row['timestamp'] = interpolated_timestamp.isoformat()
                    else:
                        # Single sample: use current timestamp
                        row['timestamp'] = current_timestamp.isoformat()
                    row['pressure'] = pressure
                    writer.writerow(row)
                    total_rows += 1
            else:
                # If no pressure samples, write one row with empty pressure
                row = base_row.copy()
                row['timestamp'] = current_timestamp.isoformat()
                row['pressure'] = ''
                writer.writerow(row)
                total_rows += 1
    
    print(f"Exported segment with {len(segment)} log entries")
    print(f"Generated {total_rows} CSV rows (expanded by pressure samples)")
    print(f"Output written to {output_file_path}")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Extract CSV data from Zephyr flight timer log files. '
                    'Supports multiple input files and automatic segment detection.'
    )
    parser.add_argument(
        'log_files',
        nargs='+',
        help='Path(s) to the log file(s) to parse (can specify multiple files)'
    )
    parser.add_argument(
        '-o', '--output',
        help='Output CSV file path (default: flight_data.csv)',
        default='flight_data.csv'
    )
    parser.add_argument(
        '--gap-minutes',
        type=int,
        default=10,
        help='Gap in minutes to split segments (default: 10)'
    )
    parser.add_argument(
        '--segment',
        type=int,
        help='Export specific segment by index (1-based). If not specified, will prompt interactively.'
    )
    
    args = parser.parse_args()
    
    try:
        # Extract CSV data from all log files
        print(f"Processing {len(args.log_files)} log file(s)...")
        csv_lines = extract_csv_from_logs(args.log_files)
        
        if not csv_lines:
            return 1
        
        print(f"\nTotal: {len(csv_lines)} log entries extracted")
        
        # Split into segments based on timestamp gaps
        segments = split_into_segments(csv_lines, gap_minutes=args.gap_minutes)
        
        if not segments:
            print("Error: No segments found", file=sys.stderr)
            return 1
        
        print(f"\nFound {len(segments)} segment(s) (gaps > {args.gap_minutes} minutes):")
        print()
        
        # Display all segments
        for i, segment in enumerate(segments):
            print(f"  {format_segment_info(segment, i)}")
        
        print()
        
        # Determine which segment to export
        segment_index = None
        
        if args.segment is not None:
            # Use command-line argument
            segment_index = args.segment - 1  # Convert to 0-based
            if segment_index < 0 or segment_index >= len(segments):
                print(f"Error: Segment {args.segment} is out of range (1-{len(segments)})", file=sys.stderr)
                return 1
        else:
            # Interactive prompt
            while True:
                try:
                    response = input(f"Which segment to export? (1-{len(segments)}, or 'q' to quit): ").strip()
                    if response.lower() == 'q':
                        print("Cancelled.")
                        return 0
                    segment_index = int(response) - 1  # Convert to 0-based
                    if segment_index < 0 or segment_index >= len(segments):
                        print(f"Please enter a number between 1 and {len(segments)}")
                        continue
                    break
                except ValueError:
                    print("Please enter a valid number or 'q' to quit")
                except (EOFError, KeyboardInterrupt):
                    print("\nCancelled.")
                    return 0
        
        # Export the selected segment
        return export_segment_to_csv(segments[segment_index], args.output)
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
