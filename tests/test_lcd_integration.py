from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_spi_lcd_driver_and_page_are_integrated():
    syscfg = (ROOT / "mspm0-modules.syscfg").read_text(encoding="utf-8")
    main = (ROOT / "main.c").read_text(encoding="utf-8")
    page = (ROOT / "Code" / "myLCD.c").read_text(encoding="utf-8")

    assert (ROOT / "Drivers" / "LCD_SPI" / "lcd_spi.c").is_file()
    assert 'GPIO12.$name                              = "LCD"' in syscfg
    for pin in ("PB10", "PB11", "PB14", "PB26", "PB9", "PB8"):
        assert f'"{pin}"' in syscfg
    assert 'SPI1.$name                      = "SPI_LCD"' in syscfg
    assert 'SPI1.direction                  = "PICO"' in syscfg
    assert 'SPI1.targetBitRate              = 8000000' in syscfg
    assert "SPI1.peripheral.misoPin" not in syscfg
    assert "MyLCD_Init();" in main
    assert "MyLCD_Show();" in main
    assert "Grayscale_Sensor_Read();" in page
    assert "Gyro_GetAngle(&gyro_angle_deg)" in page


if __name__ == "__main__":
    test_spi_lcd_driver_and_page_are_integrated()
    print("SPI LCD integration invariants: OK")
