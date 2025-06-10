import RPi.GPIO as GPIO
import spidev
import time
import matplotlib.pyplot as plt
import matplotlib.widgets as mwidgets
import csv
from datetime import datetime

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

# Use only three single-ended channels: AIN0, AIN1, AIN2 with AINCOM as negative
SE_CHANNELS = [0, 1, 2]
NUM_CH = len(SE_CHANNELS)

LOG_FILENAME = "ads1256_single_ended_log.csv"

def wait_drdy():
    while GPIO.input(DRDY):
        time.sleep(0.0001)

def ads1256_write_cmd(spi, cmd):
    spi.xfer2([cmd])
    time.sleep(0.001)

def ads1256_write_reg(spi, reg, value):
    spi.xfer2([CMD_WREG | reg, 0x00, value])
    time.sleep(0.001)

def ads1256_set_single_channel(spi, ain):
    # Set input multiplexer to channel ain (positive), AINCOM (negative)
    mux = (ain << 4) | 0x08
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
    ads1256_write_reg(spi, 0x01, 0x08)  # MUX: AIN0/AINCOM initially (single-ended)
    ads1256_write_reg(spi, 0x02, 0x00)  # ADCON: Gain=1, Clock out off
    ads1256_write_reg(spi, 0x03, 0xF0)  # DRATE: 30kSPS
    ads1256_write_cmd(spi, CMD_SELFCAL)
    wait_drdy()

def log_data_to_file(voltage_list):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILENAME, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([now] + voltage_list)
    print(f"Logged at {now}: {voltage_list}")

def prepare_logfile_header():
    try:
        with open(LOG_FILENAME, "x", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp"] + [f"AIN{ch}-AINCOM (V)" for ch in SE_CHANNELS])
    except FileExistsError:
        pass

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
    prepare_logfile_header()

    plt.ion()
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.22)
    lines = [ax.plot([], [], label=f"AIN{ch}-AINCOM")[0] for ch in SE_CHANNELS]
    ax.legend()
    ax.set_ylim(-VREF, VREF)
    ax.set_xlim(0, 100)
    ax.set_xlabel("Sample")
    ax.set_ylabel("Voltage (V)")

    xs = list(range(100))
    ys = [[0]*100 for _ in range(NUM_CH)]
    voltages = [0.0]*NUM_CH

    # Add Start and Stop logging buttons
    start_button_ax = plt.axes([0.18, 0.07, 0.23, 0.1])
    stop_button_ax = plt.axes([0.55, 0.07, 0.23, 0.1])
    start_button = mwidgets.Button(start_button_ax, 'Start Logging', color='lightgreen', hovercolor='0.975')
    stop_button = mwidgets.Button(stop_button_ax, 'Stop Logging', color='lightcoral', hovercolor='0.975')
    is_logging = [False]

    def on_start_clicked(event):
        is_logging[0] = True
        print("Continuous logging started.")

    def on_stop_clicked(event):
        is_logging[0] = False
        print("Continuous logging stopped.")

    start_button.on_clicked(on_start_clicked)
    stop_button.on_clicked(on_stop_clicked)

    LOG_INTERVAL = 0.1  # seconds between file logs when running
    last_log_time = time.time()

    try:
        while True:
            voltages = []
            for i, ch in enumerate(SE_CHANNELS):
                ads1256_set_single_channel(spi, ch)
                wait_drdy()
                raw = ads1256_read_data(spi)
                voltage = raw_to_voltage(raw)
                voltages.append(voltage)
                ys[i].append(voltage)
                ys[i].pop(0)
                lines[i].set_data(xs, ys[i])
            ax.relim()
            ax.autoscale_view(True, True, True)
            plt.pause(0.01)

            if is_logging[0]:
                now = time.time()
                if now - last_log_time > LOG_INTERVAL:
                    log_data_to_file(voltages)
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
