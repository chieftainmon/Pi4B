import RPi.GPIO as GPIO
import spidev
import time
import matplotlib.pyplot as plt
import matplotlib.widgets as mwidgets
import csv
from datetime import datetime
from collections import deque

# Pin mapping (BCM)
DRDY = 17
RESET = 18
PDWN = 27
CS0 = 22

SPI_BUS = 0
SPI_DEVICE = 0

CMD_RDATA = 0x01
CMD_RREG = 0x10
CMD_WREG = 0x50
CMD_SDATAC = 0x0F
CMD_SELFCAL = 0xF0

VREF = 2.5  # Volts (check your board!)
GAIN = 1    # Set in ADCON register

# Full set of differential channels for ADS1256
DIFF_CHANNELS = [(0, 1), (2, 3), (4, 5), (6, 7)]
NUM_CH = len(DIFF_CHANNELS)
LOG_FILENAME_PREFIX = "ads1256_diff_log_"

# Filtering parameters
FILTER_WINDOW = 5  # Number of samples for moving average

def wait_drdy():
    while GPIO.input(DRDY):
        time.sleep(0.0001)

def ads1256_write_cmd(spi, cmd):
    spi.xfer2([cmd])
    time.sleep(0.001)

def ads1256_write_reg(spi, reg, value):
    spi.xfer2([CMD_WREG | reg, 0x00, value])
    time.sleep(0.001)

def ads1256_set_diff_channel(spi, ainp, ainm):
    mux = (ainp << 4) | (ainm & 0x0F)
    ads1256_write_reg(spi, 0x01, mux)
    time.sleep(0.001)
    ads1256_write_cmd(spi, 0xFC)  # SYNC
    time.sleep(0.001)
    ads1256_write_cmd(spi, 0x00)  # WAKEUP
    time.sleep(0.001)

def ads1256_read_data(spi):
    ads1256_write_cmd(spi, CMD_RDATA)
    time.sleep(0.001)
    raw = spi.xfer2([0xFF, 0xFF, 0xFF])
    value = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    if value & 0x800000:
        value -= 1 << 24
    return value

def raw_to_voltage(raw):
    return (raw / 0x7FFFFF) * (VREF / GAIN)

def ads1256_init(spi):
    GPIO.output(RESET, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(RESET, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET, GPIO.HIGH)
    time.sleep(0.1)
    wait_drdy()
    ads1256_write_cmd(spi, CMD_SDATAC)
    ads1256_write_reg(spi, 0x00, 0x01)  # STATUS: Auto-Calibration enabled
    ainp, ainm = DIFF_CHANNELS[0]
    mux = (ainp << 4) | (ainm & 0x0F)
    ads1256_write_reg(spi, 0x01, mux)
    ads1256_write_reg(spi, 0x02, 0x00)  # ADCON: Gain=1, Clock out off
    ads1256_write_reg(spi, 0x03, 0xF0)  # DRATE: 30kSPS
    ads1256_write_cmd(spi, CMD_SELFCAL)
    wait_drdy()

def prepare_logfile_header(log_filename):
    with open(log_filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp"] + [f"AIN{p}-AIN{n} (V)" for p, n in DIFF_CHANNELS])

def log_data_to_file(log_filename, voltage_list):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # Reduce to 6 decimal digits
    voltage_strs = [f"{v:.6f}" for v in voltage_list]
    with open(log_filename, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([now] + voltage_strs)
    print(f"Logged at {now}: {voltage_strs} to {log_filename}")

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DRDY, GPIO.IN)
    GPIO.setup(RESET, GPIO.OUT)
    GPIO.setup(PDWN, GPIO.OUT)
    GPIO.setup(CS0, GPIO.OUT)
    GPIO.output(RESET, GPIO.HIGH)
    GPIO.output(PDWN, GPIO.HIGH)
    GPIO.output(CS0, GPIO.LOW)

    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = 1000000
    spi.mode = 1

    ads1256_init(spi)

    plt.ion()
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.36)
    lines = [ax.plot([], [], label=f"AIN{p}-AIN{n}")[0] for p, n in DIFF_CHANNELS]
    ax.legend()
    ax.set_ylim(-VREF, VREF)
    ax.set_xlim(0, 100)
    ax.set_xlabel("Sample")
    ax.set_ylabel("Voltage (V)")

    xs = list(range(100))
    ys = [[0]*100 for _ in range(NUM_CH)]
    voltages = [0.0]*NUM_CH

    # Filtering buffer and state
    buffers = [deque([0.0]*FILTER_WINDOW, maxlen=FILTER_WINDOW) for _ in range(NUM_CH)]
    is_filtering = [True]

    # Add Start, Stop, Zero, and Filter Toggle buttons
    start_button_ax = plt.axes([0.03, 0.07, 0.16, 0.1])
    stop_button_ax = plt.axes([0.21, 0.07, 0.16, 0.1])
    zero_button_ax = plt.axes([0.39, 0.07, 0.16, 0.1])
    filter_button_ax = plt.axes([0.57, 0.07, 0.24, 0.1])
    start_button = mwidgets.Button(start_button_ax, 'Start Logging', color='lightgreen', hovercolor='0.975')
    stop_button = mwidgets.Button(stop_button_ax, 'Stop Logging', color='lightcoral', hovercolor='0.975')
    zero_button = mwidgets.Button(zero_button_ax, 'Zero Offset', color='lightblue', hovercolor='0.975')
    filter_button = mwidgets.Button(filter_button_ax, 'Toggle Filtering', color='khaki', hovercolor='0.975')

    is_logging = [False]
    log_filename = [None]
    offset = [[0.0]*NUM_CH]  # Outer list for mutability in closures

    def on_start_clicked(event):
        is_logging[0] = True
        file_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename[0] = LOG_FILENAME_PREFIX + file_time + ".csv"
        prepare_logfile_header(log_filename[0])
        print(f"Continuous logging started. Logging to {log_filename[0]}")

    def on_stop_clicked(event):
        is_logging[0] = False
        print("Continuous logging stopped.")

    def on_zero_clicked(event):
        offset[0] = voltages.copy()
        print(f"Offset set to: {offset[0]} (data will be zeroed to this)")

    def on_filter_clicked(event):
        is_filtering[0] = not is_filtering[0]
        state = "ON" if is_filtering[0] else "OFF"
        print(f"Filtering toggled {state}")

    start_button.on_clicked(on_start_clicked)
    stop_button.on_clicked(on_stop_clicked)
    zero_button.on_clicked(on_zero_clicked)
    filter_button.on_clicked(on_filter_clicked)

    LOG_INTERVAL = 0.1  # seconds between file logs when running
    last_log_time = time.time()

    try:
        while True:
            voltages.clear()
            for i, (ainp, ainm) in enumerate(DIFF_CHANNELS):
                ads1256_set_diff_channel(spi, ainp, ainm)
                wait_drdy()
                raw = ads1256_read_data(spi)
                voltage = raw_to_voltage(raw) - offset[0][i]
                buffers[i].append(voltage)
                # Apply filtering if enabled
                if is_filtering[0]:
                    filtered_voltage = sum(buffers[i]) / len(buffers[i])
                else:
                    filtered_voltage = voltage
                # Reduce to 6 decimals for display
                filtered_voltage = round(filtered_voltage, 6)
                voltages.append(filtered_voltage)
                ys[i].append(filtered_voltage)
                ys[i].pop(0)
                lines[i].set_data(xs, ys[i])
            ax.relim()
            ax.autoscale_view(True, True, True)
            plt.pause(0.01)

            if is_logging[0] and log_filename[0] is not None:
                now = time.time()
                if now - last_log_time > LOG_INTERVAL:
                    log_data_to_file(log_filename[0], voltages)
                    last_log_time = now
            else:
                last_log_time = time.time()  # Reset timer so logging resumes cleanly

    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
