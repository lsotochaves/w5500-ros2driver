import sys
import re
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

if len(sys.argv) != 2:
    print("Usage: python3 plot_forces.py <csv_file>")
    sys.exit(1)

FILE_PATH = sys.argv[1]

# Extract port number from filename (last 4 digits before .csv)
match = re.search(r'_(\d{4})\.csv$', FILE_PATH)
PORT = match.group(1) if match else "Unknown"

def read_data(file_path):
    try:
        data = pd.read_csv(file_path)
        for column in data.columns[1:]:
            data[column] = pd.to_numeric(data[column], errors="coerce")
        return data.dropna()
    except Exception:
        return pd.DataFrame()

fig, ax = plt.subplots()

def init():
    ax.set_title(f"Port {PORT}")
    ax.set_xlabel("")
    ax.set_ylabel("Force Values")
    return []

def update(frame):
    data = read_data(FILE_PATH)
    if data.empty:
        return []

    ax.clear()
    ax.set_title(f"Port {PORT}")
    ax.set_xlabel("")
    ax.set_ylabel("Force Values")

    if "Batch Number" in data.columns:
        data = data.drop(columns=["Batch Number"])

    x_values = range(len(data))
    for column in data.columns[1:]:
        ax.plot(x_values, data[column], label=column)

    ax.legend(loc="upper right")
    return []

ani = FuncAnimation(fig, update, init_func=init, interval=500)
plt.show()
