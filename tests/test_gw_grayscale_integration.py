from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_gw_grayscale_replaces_uart_path():
    syscfg = (ROOT / "mspm0-modules.syscfg").read_text(encoding="utf-8")
    main = (ROOT / "main.c").read_text(encoding="utf-8")
    oled = (ROOT / "Code" / "myOLED.c").read_text(encoding="utf-8")
    task = (ROOT / "Code" / "myTask.c").read_text(encoding="utf-8")
    adapter = (ROOT / "Code" / "Grayscale_Sensor.c").read_text(encoding="utf-8")
    adapter_header = (ROOT / "Code" / "Grayscale_Sensor.h").read_text(
        encoding="utf-8"
    )

    assert (ROOT / "Drivers" / "GW_Grayscale" / "gw_grayscale_mspm0.c").is_file()
    assert 'GPIO10.$name                          = "SCL"' in syscfg
    assert 'GPIO10.associatedPins[0].assignedPin  = "15"' in syscfg
    assert 'GPIO11.$name                          = "SDA"' in syscfg
    assert 'GPIO11.associatedPins[0].assignedPin  = "5"' in syscfg
    assert "UART_GRAY" not in syscfg
    assert not (ROOT / "Code" / "grayscale_uart.c").exists()

    assert "Grayscale_Sensor_Init();" in main
    assert "black_mask = Grayscale_Sensor_Read();" in oled
    assert "black_mask = Grayscale_Sensor_Read();" in task
    assert "GW_Gray_Serial_Read(&g_gray_serial)" in adapter
    assert "#define GRAYSCALE_BLACK_LEVEL (0U)" in adapter_header
    assert "black_mask = (uint8_t)~black_mask;" in adapter


if __name__ == "__main__":
    test_gw_grayscale_replaces_uart_path()
    print("GW grayscale integration invariants: OK")
