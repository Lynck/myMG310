"""VL53L0X距离链路的最小静态回归检查。"""

from pathlib import Path


ROOT = Path(__file__).parents[1]
syscfg = (ROOT / "mspm0-modules.syscfg").read_text(encoding="utf-8")
distance = (ROOT / "Code" / "leader_distance.c").read_text(encoding="utf-8")
oled = (ROOT / "Code" / "myOLED.c").read_text(encoding="utf-8")
project = (ROOT / ".cproject").read_text(encoding="utf-8")
oled_hw = (
    ROOT / "Drivers" / "OLED_Hardware_I2C" / "oled_hardware_i2c.c"
).read_text(encoding="utf-8")

assert 'I2C1.$name                             = "I2C_VL53L0X"' in syscfg
assert 'I2C1.peripheral.$assign                = "I2C0"' in syscfg
assert 'I2C1.peripheral.sdaPin.$assign         = "PA0"' in syscfg
assert 'I2C1.peripheral.sclPin.$assign         = "PA1"' in syscfg
assert 'GPIO1.$name                          = "GPIO_OLED"' not in syscfg
assert "UART_DISTANCE" not in syscfg
assert "VL53L0X_Process()" in distance
assert "(float)distance_mm * 0.1f" in distance
assert '"TOF:%6.1fcm"' in oled
assert '#include "oled_hardware_i2c.h"' in oled
source_exclusion = next(line for line in project.splitlines() if "<entry excluding=" in line)
assert "Drivers/OLED_Software_I2C" in source_exclusion
assert "Drivers/OLED_Hardware_I2C" not in source_exclusion
assert "I2C_VL53L0X_INST" in oled_hw
assert "I2C_OLED_INST" not in oled_hw

print("VL53L0X integration invariants: OK")
