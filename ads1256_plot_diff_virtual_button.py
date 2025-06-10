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

VREF = 2.5  # Volts (check your board!)
GAIN = 1    # Set in ADCON register

DIFF_PAIRS = [(0,1), (2,3), (4,5), (6,7)]
NUM_DIFF = len(DIFF_PAIRS)

LOG_FILENAME = "ads1256_diff_log.csv"

def wait_drdy():
    while GPIO.input(DRDY):
        time.sleep(0.0001)

def ads1256_write_cmd(spi, cmd):
    spi.xfer2([cmd])
    time.sleep(0.001)

def ads1256_write_reg(spi, reg, value):
    spi.xfer2([CMD_WREG | reg, 0x00, value])
    time.sleep(0.001)

def ads1256_set_diff_channel(spi, ainp, ainn):
    mux = (ainp << 4) | ainn
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
    ads1256_write_reg(spi, 0x01, 0x01)  # MUX: AIN0/AIN1 initially (differential)
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
            writer.writerow(["timestamp"] + [f"AIN{a}-AIN{b} (V)" for a, b in DIFF_PAIRS])
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
    plt.subplots_adjust(bottom=0.2)
    lines = [ax.plot([], [], label=f"AIN{ainp}-AIN{ainn}")[0] for ainp, ainn in DIFF_PAIRS]
    ax.legend()
    ax.set_ylim(-VREF, VREF)
    ax.set_xlim(0, 100)
    ax.set_xlabel("Sample")
    ax.set_ylabel("Voltage (V)")

    xs = list(range(100))
    ys = [[0]*100 for _ in range(NUM_DIFF)]
    last_logged_time = 0
    voltages = [0.0]*NUM_DIFF

    # Add virtual button using matplotlib.widgets.Button
    button_ax = plt.axes([0.35, 0.05, 0.3, 0.08])
    log_button = mwidgets.Button(button_ax, 'Log Current Data', color='lightgoldenrodyellow', hovercolor='0.975')
    log_button_clicked = [False]

    def on_button_clicked(event):
        log_button_clicked[0] = True

    log_button.on_clicked(on_button_clicked)

    try:
        while True:
            voltages = []
            for i, (ainp, ainn) in enumerate(DIFF_PAIRS):
                ads1256_set_diff_channel(spi, ainp, ainn)
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

            if log_button_clicked[0]:
                # Debounce: prevent multiple logs in quick succession
                if time.time() - last_logged_time > 0.5:
                    log_data_to_file(voltages)
                    last_logged_time = time.time()
                log_button_clicked[0] = False

    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()