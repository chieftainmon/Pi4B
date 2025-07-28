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

VREF = 5  # Volts (check your board!)
GAIN = 1    # Set in ADCON register

# Full set of differential channels for ADS1256
DIFF_CHANNELS = [(0, 1), (2, 3), (4, 5), (6, 7)]
NUM_CH = len(DIFF_CHANNELS)
LOG_FILENAME_PREFIX = "ads1256_diff_log_"

# Filtering parameters
FILTER_WINDOW = 5  # Number of samples for moving average

def voltage_to_force(voltage):
    # 5 V corresponds to 120 kN
    return (voltage / 5.0) * 120.0

def voltage_to_torque(voltage):
    # 5 V corresponds to 100 Nm
    return (voltage / 5.0) * 100.0

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

def prepare_logfile_header(log_filename):
    with open(log_filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp",
            "Force (kN)",    # AIN0-AIN1
            "Torque (Nm)",   # AIN2-AIN3
            "AIN4-AIN5 (V)",
            "AIN6-AIN7 (V)"
        ])

def log_data_to_file(log_filename, value_list):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    value_strs = [f"{v:.6f}" for v in value_list]
    with open(log_filename, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([now] + value_strs)
    print(f"Logged at {now}: {value_strs} to {log_filename}")

def main():
    # GPIO & SPI setup
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

    # Plot setup: two y-axes, only Force and Torque
    plt.ion()
    fig, ax1 = plt.subplots()
    plt.subplots_adjust(bottom=0.36)

    xs = list(range(100))
    ys_force = [0]*100
    ys_torque = [0]*100

    # Force on left y-axis
    line_force, = ax1.plot(xs, ys_force, 'b-', label="Force (kN)")
    ax1.set_xlabel("Sample")
    ax1.set_ylabel("Force (kN)", color="b")
    ax1.set_ylim(-10, 130)  # Adjust limits as needed
    ax1.tick_params(axis='y', labelcolor='b')

    # Torque on right y-axis
    ax2 = ax1.twinx()
    line_torque, = ax2.plot(xs, ys_torque, 'r-', label="Torque (Nm)")
    ax2.set_ylabel("Torque (Nm)", color="r")
    ax2.set_ylim(-10, 130)  # Adjust limits as needed
    ax2.tick_params(axis='y', labelcolor='r')

    # Combine legend for both
    fig.legend([line_force, line_torque], ["Force (kN)", "Torque (Nm)"], loc='upper right')

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
    filter_button = mwidgets.Button(filter_button_ax, 'Toggle Filter', color='wheat', hovercolor='0.975')

    is_logging = [False]
    log_filename = [None]
    offset = [[0.0]*NUM_CH]

    def on_start(event):
        if not is_logging[0]:
            log_filename[0] = LOG_FILENAME_PREFIX + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
            prepare_logfile_header(log_filename[0])
            is_logging[0] = True
            print(f"Started logging to {log_filename[0]}")

    def on_stop(event):
        if is_logging[0]:
            is_logging[0] = False
            print(f"Stopped logging to {log_filename[0]}")

    def on_zero(event):
        print("Zeroing offset. Please make sure sensors are unloaded.")
        for i in range(NUM_CH):
            offset[0][i] = 0.0
        # Read a few samples for offset averaging
        for n in range(10):
            for i, (ainp, ainm) in enumerate(DIFF_CHANNELS):
                ads1256_set_diff_channel(spi, ainp, ainm)
                wait_drdy()
                raw = ads1256_read_data(spi)
                voltage = raw_to_voltage(raw)
                offset[0][i] += voltage / 10.0
        print(f"New offset: {offset[0]}")

    def on_filter_toggle(event):
        is_filtering[0] = not is_filtering[0]
        print(f"Filtering {'enabled' if is_filtering[0] else 'disabled'}")

    start_button.on_clicked(on_start)
    stop_button.on_clicked(on_stop)
    zero_button.on_clicked(on_zero)
    filter_button.on_clicked(on_filter_toggle)

    # Main acquisition & plotting loop
    last_log_time = time.time()
    LOG_INTERVAL = 0.2  # seconds

    try:
        while True:
            voltages = []
            physical_values = []
            value_force = None
            value_torque = None
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
                filtered_voltage = round(filtered_voltage, 6)
                voltages.append(filtered_voltage)
                # Convert for plotting and logging
                if i == 0:
                    value = voltage_to_force(filtered_voltage)
                    value_force = value
                elif i == 1:
                    value = voltage_to_torque(filtered_voltage)
                    value_torque = value
                else:
                    value = filtered_voltage
                value = round(value, 6)
                physical_values.append(value)

            # Update plotting buffers only for Force and Torque
            ys_force.append(value_force)
            ys_force.pop(0)
            ys_torque.append(value_torque)
            ys_torque.pop(0)
            line_force.set_data(xs, ys_force)
            line_torque.set_data(xs, ys_torque)
            ax1.set_xlim(0, 100)
            ax2.set_xlim(0, 100)
            ax1.relim()
            ax1.autoscale_view(True, True, True)
            ax2.relim()
            ax2.autoscale_view(True, True, True)
            plt.pause(0.01)

            # Logging (all 4 channels)
            if is_logging[0] and log_filename[0] is not None:
                now = time.time()
                if now - last_log_time > LOG_INTERVAL:
                    log_data_to_file(log_filename[0], physical_values)
                    last_log_time = now
            else:
                last_log_time = time.time()  # reset so logging resumes cleanly
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
