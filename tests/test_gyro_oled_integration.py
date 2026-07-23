"""单轴陀螺仪 UART 接收和 OLED 显示的最小静态回归检查。"""

from pathlib import Path


ROOT = Path(__file__).parents[1]
syscfg = (ROOT / "mspm0-modules.syscfg").read_text(encoding="utf-8")
main = (ROOT / "main.c").read_text(encoding="utf-8")
oled = (ROOT / "Code" / "myOLED.c").read_text(encoding="utf-8")

assert 'UART3.$name                            = "UART_GYRO"' in syscfg
assert 'UART3.enabledInterrupts                = ["RX"]' in syscfg
assert 'UART3.targetBaudRate                   = 115200' in syscfg
assert '#include "Drivers/SingleAxisGyro/single_axis_gyro.h"' in main
assert "SingleAxisGyro_Init(&gyro, NULL, NULL, NULL);" in main
assert "NVIC_EnableIRQ(UART_GYRO_INST_INT_IRQN);" in main
assert "void UART_GYRO_INST_IRQHandler(void)" in main
assert "SingleAxisGyro_ReceiveByte(" in main
assert "Gyro_GetAngle(&gyro_angle_deg)" in oled
assert 'sprintf(text, "Yaw:%7.2f deg", gyro_angle_deg);' in oled
assert "OLED_ShowString(0, 7" in oled
assert "black_mask = Grayscale_Sensor_Read();" in oled
assert "(black_mask & 0x80U) ? '1' : '0'" in oled
assert "(black_mask & 0x01U) ? '1' : '0'" in oled

print("Gyroscope OLED integration invariants: OK")
