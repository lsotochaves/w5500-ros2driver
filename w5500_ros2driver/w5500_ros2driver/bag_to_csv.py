#!/usr/bin/env python3
"""
Export ROS2 bag data to CSV files
Usage: python3 bag_to_csv.py <bag_directory>
Example: python3 bag_to_csv.py my_recording
"""

import sys
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from w5500_msg.msg import ForceF4
import csv


def export_bag_to_csv(bag_path, output_dir='./csv_output'):
    """Export each topic in bag to separate CSV file"""
    
    # Create output directory
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    # Detect storage format automatically
    if str(bag_path).endswith(".db3"):
        storage_id = "sqlite3"
    elif str(bag_path).endswith(".mcap"):
        storage_id = "mcap"
    else:
        # fallback: try sqlite3 first
        storage_id = "sqlite3"

    storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Dictionary to hold CSV writers for each topic
    csv_files = {}
    csv_writers = {}
    msg_count = {}
    
    print(f"Reading bag: {bag_path}")
    print(f"Detected storage format: {storage_id}")
    print(f"Output directory: {output_dir}")
    
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            # Only process Force topics
            if not topic.endswith('/force'):
                continue
            
            # Create CSV file for this topic if not exists
            if topic not in csv_files:
                # Clean topic name for filename
                filename = topic.replace('/', '_').lstrip('_') + '.csv'
                filepath = Path(output_dir) / filename
                
                csv_files[topic] = open(filepath, 'w', newline='')
                csv_writers[topic] = csv.writer(csv_files[topic])
                
                # Write header
                csv_writers[topic].writerow([
                    'timestamp_ns',   # tiempo del bag (grabación)
                    'msg_time_s',     # tiempo absoluto en segundos del mensaje
                    'info',
                    'fx_my_1',
                    'fy_mx_1',
                    'fy_mx_2',
                    'fx_my_2',
                    'mz',
                    'fz_1',
                    'fz_2'
                ])
                
                msg_count[topic] = 0
                print(f"Created CSV for topic: {topic}")
            
            # Deserialize message
            msg = deserialize_message(data, ForceF4)

            # Combinar sec + nanosec en segundos (float)
            msg_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            
            # Write data row
            csv_writers[topic].writerow([
                timestamp,       # nanosegundos del bag
                msg_time,        # segundos (float) del publicador
                msg.info,
                msg.fx_my_1,
                msg.fy_mx_1,
                msg.fy_mx_2,
                msg.fx_my_2,
                msg.mz,
                msg.fz_1,
                msg.fz_2
            ])
            
            msg_count[topic] += 1
            
            # Progress indicator
            if msg_count[topic] % 10000 == 0:
                print(f"  {topic}: {msg_count[topic]} messages processed...")
    
    finally:
        # Close all CSV files
        for f in csv_files.values():
            f.close()
    
    print("\n=== Export Complete ===")
    for topic, count in msg_count.items():
        filename = topic.replace('/', '_').lstrip('_') + '.csv'
        print(f"{filename}: {count} messages")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 bag_to_csv.py <bag_directory> [output_dir]")
        print("Example: python3 bag_to_csv.py my_recording")
        print("Example: python3 bag_to_csv.py my_recording ./my_csv_files")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else './csv_output'
    
    if not Path(bag_path).exists():
        print(f"Error: Bag directory '{bag_path}' does not exist")
        sys.exit(1)
    
    export_bag_to_csv(bag_path, output_dir)


if __name__ == '__main__':
    main()
