import serial
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm
import time

PORT = "COM7"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

freqS = []
magS = []

note=""
cents=""

latest_frame = None

plt.ion()
fig, ax = plt.subplots()

last_plot = 0
plot_interval = 0.05

while True:
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        if line[:1] == "N":
            try:
                identifier, note, cents = line.split(",")

            except ValueError:
                pass

        else:
            latest_frame = line

    current_time = time.monotonic()

    if latest_frame is not None and current_time - last_plot >= plot_interval:

        try:
            freqS.clear()
            magS.clear()
            notes = latest_frame.split(";")

            for entry in notes:
                if not entry:
                    continue

                freq, mag = entry.split(",")
                freqS.append(freq)
                magS.append(float(mag))

            if magS:

                ax.clear()
                norm = colors.Normalize(vmin=0, vmax=2000)
                bar_colors = cm.plasma(norm(magS))

                ax.bar(freqS, magS, color=bar_colors)

                ax.set_ylim(0, max(2000, max(magS) * 1.2))

                ax.set_xlabel("Note")
                ax.set_ylabel("Mag")

                ax.set_title("Primary Note: " + note + "   ||   Cents: " + cents)

                plt.xticks(rotation=90)
                plt.tight_layout()

                plt.pause(0.001)

            latest_frame = None
            last_plot = current_time

        except ValueError:
            latest_frame = None

    else:
        plt.pause(0.001)
