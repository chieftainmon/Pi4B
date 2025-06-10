import RPi.GPIO as GPIO
import spidev
import time
import matplotlib.pyplot as plt
import matplotlib.widgets as mwidgets

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
    plt.subplots_adjust(bottom=0.2)
    lines = [ax.plot([], [], label=f"AIN{ainp}-AIN{ainn}")[0] for ainp, ainn in DIFF_PAIRS]
    ax.legend()
    ax.set_ylim(-VREF, VREF)
    ax.set_xlim(0, 100)
    ax.set_xlabel("Sample")
    ax.set_ylabel("Voltage (V)")

    xs = list(range(100))
    ys = [[0]*100 for _ in range(NUM_DIFF)]

    N_CYCLES = 100  # How often to print sampling rate
    cycle_count = 0
    t_start = time.time()

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

            cycle_count += 1
            if cycle_count == N_CYCLES:
                t_end = time.time()
                elapsed = t_end - t_start
                rate_per_channel = N_CYCLES / elapsed
                print(f"Effective sampling rate: {rate_per_channel:.2f} samples/sec/channel over {N_CYCLES} cycles")
                cycle_count = 0
                t_start = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()