import RPi.GPIO as GPIO
import spidev
import time

# GPIO PIN ASSIGNMENTS (BCM numbering)
BUTTON_FWD = 20     # Forward button
BUTTON_BWD = 21     # Backward button
STEPPER_STEP = 6    # Step pin for driver
STEPPER_DIR = 13    # Direction pin for driver

# ADS1256 SPI pins (as used in earlier code)
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

VREF = 2.5  # Adjust if your board uses a different ref voltage
GAIN = 1

AN0_CHANNEL = 0  # Potentiometer on AIN0 (vs AINCOM)

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
    mux = (ain << 4) | 0x08  # AINx vs AINCOM
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
    ads1256_write_reg(spi, 0x01, 0x08)  # MUX: AIN0/AINCOM
    ads1256_write_reg(spi, 0x02, 0x00)  # ADCON: Gain=1, Clock out off
    ads1256_write_reg(spi, 0x03, 0xF0)  # DRATE: 30kSPS
    ads1256_write_cmd(spi, CMD_SELFCAL)
    wait_drdy()

def read_potentiometer(spi):
    ads1256_set_single_channel(spi, AN0_CHANNEL)
    wait_drdy()
    raw = ads1256_read_data(spi)
    voltage = raw_to_voltage(raw)
    # Map voltage (0 - VREF) to speed (e.g., min 0.5ms delay, max 10ms delay)
    # Clamp for safety
    v = max(0.0, min(voltage, VREF))
    min_delay = 0.0005  # Fastest step = 500us
    max_delay = 0.01    # Slowest step = 10ms
    delay = max_delay - (v / VREF) * (max_delay - min_delay)
    return delay

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DRDY, GPIO.IN)
    GPIO.setup(RESET, GPIO.OUT)
    GPIO.setup(PDWN, GPIO.OUT)
    GPIO.setup(CS0, GPIO.OUT)
    GPIO.setup(BUTTON_FWD, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUTTON_BWD, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(STEPPER_STEP, GPIO.OUT)
    GPIO.setup(STEPPER_DIR, GPIO.OUT)
    GPIO.output(RESET, GPIO.HIGH)
    GPIO.output(PDWN, GPIO.HIGH)
    GPIO.output(CS0, GPIO.LOW)
    GPIO.output(STEPPER_STEP, GPIO.LOW)
    GPIO.output(STEPPER_DIR, GPIO.LOW)

    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = 1000000
    spi.mode = 1

    ads1256_init(spi)

    print("Press and hold a button to move stepper. Adjust potentiometer to change speed.")

    try:
        while True:
            # Button: Active low
            fwd = GPIO.input(BUTTON_FWD) == 0
            bwd = GPIO.input(BUTTON_BWD) == 0
            delay = read_potentiometer(spi)

            if fwd:
                GPIO.output(STEPPER_DIR, GPIO.HIGH)
                GPIO.output(STEPPER_STEP, GPIO.HIGH)
                time.sleep(delay/2)
                GPIO.output(STEPPER_STEP, GPIO.LOW)
                time.sleep(delay/2)
            elif bwd:
                GPIO.output(STEPPER_DIR, GPIO.LOW)
                GPIO.output(STEPPER_STEP, GPIO.HIGH)
                time.sleep(delay/2)
                GPIO.output(STEPPER_STEP, GPIO.LOW)
                time.sleep(delay/2)
            else:
                time.sleep(0.01)  # Idle, debounce

    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
