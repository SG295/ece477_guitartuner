import serial
import numpy as np
import matplotlib.pyplot as plt

PORT = 'COM3'      # Update to match your COM port
BAUD = 115200
BUFFER_SIZE = 1024
SAMPLE_RATE = 42000

ser = serial.Serial(PORT, BAUD, timeout=1)
print("Reading from", PORT)

# Setup the plots
plt.ion()  # interactive mode ON
fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(10, 6))

time_line, = ax_time.plot([], [], label="Time Domain")
freq_line, = ax_freq.plot([], [], label="Frequency Domain")

ax_time.set_xlim(0, BUFFER_SIZE)
ax_time.set_ylim(-1.1, 1.1)
ax_time.set_title("Mic Input - Time Domain")

ax_freq.set_xlim(0, SAMPLE_RATE)
ax_freq.set_ylim(0, 80)  # adjust as needed
ax_freq.set_title("Mic Input - Frequency Domain")

while True:
    samples = []
    while len(samples) < BUFFER_SIZE:
        try:
            line = ser.readline().decode().strip()
            if line:
                val = int(line)
                samples.append(val)
        except Exception as e:
            print("Parse error:", e)

    # Convert to float and normalize for plotting
    samples = np.array(samples, dtype=np.float32) / (pow(2,17))

    # Update time-domain plot
    time_line.set_ydata(samples)
    time_line.set_xdata(np.arange(len(samples)))

    # Update FFT plot
    fft_out = np.fft.rfft(samples * np.hanning(len(samples)))
    freqs = np.fft.rfftfreq(len(samples), 1 / SAMPLE_RATE)
    mag = np.abs(fft_out)

    freq_line.set_ydata(mag)
    freq_line.set_xdata(freqs)

    ax_freq.set_xlim(0, SAMPLE_RATE)
    ax_freq.set_ylim(0, np.max(mag) * 1.2)

    fig.canvas.draw()
    fig.canvas.flush_events()
