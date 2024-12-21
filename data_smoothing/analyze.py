import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, butter, filtfilt
import csv


def read_data(csv_file, c1, c2):
    differences = []

    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        headers = next(reader)  # Skip header row

        for row in reader:
            difference = float(row[c1]) - float(row[c2])
            differences.append(difference)

    return differences

def moving_average(data, window):
    """Simple moving average filter"""
    return np.convolve(data, np.ones(window)/window, mode='valid')

def butterworth_lowpass(data, cutoff, fs, order=5):
    """Butterworth low-pass filter"""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)


def gaussian_filter_custom(signal, sigma):
    # Create Gaussian kernel
    kernel_size = int(6 * sigma + 1)  # Typically 6σ captures most of the distribution
    kernel = np.exp(-np.linspace(-3, 3, kernel_size) ** 2 / (2 * sigma ** 2))
    kernel /= np.sum(kernel)

    # Convolution
    return np.convolve(signal, kernel, mode='same')


t = np.linspace(0, 2000, 2000)
noisy_signal = np.array(read_data("rotation_tests2.csv", 2, 3))[:2000]
# Apply different filters
ma_filtered = moving_average(noisy_signal, window=20)
sg_filtered = savgol_filter(noisy_signal, window_length=51, polyorder=3)
bw_filtered = butterworth_lowpass(noisy_signal, cutoff=2, fs=200)
gaussian_filtered = gaussian_filter_custom(noisy_signal, sigma=5)

#noisy_signal = np.diff(bw_filtered, n=1, prepend=0)

# Plotting
plt.figure(figsize=(12, 8))
plt.plot(t, noisy_signal, label='Original Noisy Signal', alpha=0.5)
#plt.plot(t[:len(ma_filtered)], ma_filtered, label='Moving Average', linewidth=2)
#plt.plot(t, sg_filtered, label='Savitzky-Golay', linewidth=2)
plt.plot(t, gaussian_filtered, label='gaussian', linewidth=2)
plt.plot(t, bw_filtered, label='Butterworth', linewidth=2)
plt.title('Signal Filtering Comparison')
plt.legend()
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.show()