import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy import signal
import argparse
import struct
import time

class AudioVisualizer:
    def __init__(self, port, baudrate=115200, buffer_size=1024, sample_rate=32000):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        self.sample_rate = sample_rate
        
        # Initialize serial connection
        self.ser = None
        self.connect_serial()
        
        # Data buffers
        self.audio_buffer = np.zeros(buffer_size)
        self.spectrum_buffer = np.zeros(buffer_size // 2)
        
        # Setup visualization
        self.setup_plot()
    
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            exit(1)
    
    def setup_plot(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Time domain plot
        self.line1, = self.ax1.plot([], [], lw=1)
        self.ax1.set_xlim(0, self.buffer_size)
        self.ax1.set_ylim(-32768, 32767)  # 16-bit audio range
        self.ax1.set_title('Time Domain')
        self.ax1.set_ylabel('Amplitude')
        self.ax1.grid(True)
        
        # Frequency domain plot
        self.line2, = self.ax2.plot([], [], lw=1)
        self.freq = np.fft.rfftfreq(self.buffer_size, d=1/self.sample_rate)
        self.ax2.set_xlim(0, self.sample_rate / 2)
        self.ax2.set_ylim(0, 100)  # Will be adjusted dynamically
        self.ax2.set_title('Frequency Spectrum')
        self.ax2.set_xlabel('Frequency (Hz)')
        self.ax2.set_ylabel('Magnitude (dB)')
        self.ax2.grid(True)
        
        plt.tight_layout()
    
    def read_data(self):
        # Check if data is available
        if self.ser.in_waiting < 2 * self.buffer_size:
            return False
        
        # Read data (each audio sample is 2 bytes)
        data = self.ser.read(2 * self.buffer_size)
        
        # Convert bytes to 16-bit audio samples
        # Assuming data is sent as little-endian 16-bit integers
        samples = []
        for i in range(0, len(data), 2):
            if i + 1 < len(data):
                # Convert 2 bytes to a 16-bit integer
                sample = struct.unpack('<h', data[i:i+2])[0]
                samples.append(sample)
        
        if len(samples) == self.buffer_size:
            self.audio_buffer = np.array(samples)
            return True
        
        return False
    
    def update_plot(self, frame):
        if self.read_data():
            # Update time domain plot
            self.line1.set_data(np.arange(len(self.audio_buffer)), self.audio_buffer)
            
            # Compute frequency spectrum
            # Apply window to reduce spectral leakage
            windowed = self.audio_buffer * signal.windows.hann(len(self.audio_buffer))
            spectrum = np.abs(np.fft.rfft(windowed))
            
            # Convert to dB scale (with noise floor)
            spectrum_db = 20 * np.log10(spectrum + 1e-10)
            
            # Update frequency domain plot
            self.line2.set_data(self.freq, spectrum_db)
            
            # Dynamically adjust y-axis for spectrum
            self.ax2.set_ylim(np.min(spectrum_db), np.max(spectrum_db) + 10)
        
        return self.line1, self.line2
    
    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot, blit=True, interval=50)
        plt.show()
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize audio data from STM32 via UART')
    parser.add_argument('--port', default="COM3", help='Serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--buffer', type=int, default=1024, help='Buffer size (default: 1024)')
    parser.add_argument('--rate', type=int, default=32000, help='Sample rate in Hz (default: 32000)')
    
    args = parser.parse_args()
    
    visualizer = AudioVisualizer(args.port, args.baudrate, args.buffer, args.rate)
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("Visualization stopped")
    finally:
        visualizer.close()