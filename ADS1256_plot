import RPi.GPIO as GPIO
import spidev
import time
import matplotlib.pyplot as plt

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

CHANNELS = 8

def wait_drdy():
    while GPIO.input(DRDY):
        time.sleep(0.0001)

def ads1256_write_cmd(spi, cmd):
    spi.xfer2([cmd])
    time.sleep(0.001)

def ads1256_write_reg(spi, reg, value):
    spi.xfer2([CMD_WREG | reg, 0x00, value])
    time.sleep(0.001)

def ads1256_set_channel(spi, ch):
    # Set input multiplexer to channel ch (positive), AINCOM (negative)
    ads1256_write_reg(spi, 0x01, (ch << 4) | 0x08)
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
    ads1256_write_reg(spi, 0x01, 0x08)  # MUX: AIN0/AINCOM initially
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
    lines = [ax.plot([], [], label=f"CH{i}")[0] for i in range(CHANNELS)]
    ax.legend()
    ax.set_ylim(-8388608, 8388607)
    ax.set_xlim(0, 100)
    ax.set_xlabel("Sample")
    ax.set_ylabel("ADC Value")

    xs = list(range(100))
    ys = [[0]*100 for _ in range(CHANNELS)]

    try:
        while True:
            for ch in range(CHANNELS):
                ads1256_set_channel(spi, ch)
                wait_drdy()
                value = ads1256_read_data(spi)
                ys[ch].append(value)
                ys[ch].pop(0)
                lines[ch].set_data(xs, ys[ch])
            ax.relim()
            ax.autoscale_view(True, True, True)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
