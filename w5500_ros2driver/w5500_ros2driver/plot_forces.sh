#!/bin/bash

# List of CSV files
FILES=(
    "$HOME/output_w5500_1_5000.csv"
    "$HOME/output_w5500_2_5001.csv"
    "$HOME/output_w5500_3_5002.csv"
    "$HOME/output_w5500_4_5003.csv"
)

# Launch each plot in parallel
for FILE in "${FILES[@]}"; do
    python3 plot_forces.py "$FILE" &
done

wait
