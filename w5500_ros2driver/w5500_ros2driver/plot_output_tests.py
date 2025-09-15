import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

FILE_PATH = r"~/output_w5500_1_5000.csv"

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
    ax.set_title("Real-Time Plot of Forces")
    ax.set_xlabel("")  # leave x-axis label blank
    ax.set_ylabel("Force Values")
    return []

def update(frame):
    data = read_data(FILE_PATH)

    if data.empty:
        return []

    ax.clear()  # clear previous frame
    ax.set_title("Real-Time Plot of Forces")
    ax.set_xlabel("")
    ax.set_ylabel("Force Values")

        # Drop Batch Number if it exists
    if "Batch Number" in data.columns:
        data = data.drop(columns=["Batch Number"])

    x_values = range(len(data))
    for column in data.columns[1:]:
        ax.plot(x_values, data[column], label=column)

    ax.legend(loc="upper right")
    return []

ani = FuncAnimation(fig, update, init_func=init, interval=500)  # update every 0.5s
plt.show()
