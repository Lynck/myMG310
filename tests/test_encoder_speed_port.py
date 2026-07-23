from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_encoder_speed_driver_matches_new_board():
    header = (ROOT / "Code" / "motor_speed.h").read_text(encoding="utf-8")
    source = (ROOT / "Code" / "motor_speed.c").read_text(encoding="utf-8")
    interrupts = (ROOT / "Drivers" / "MSPM0" / "interrupt.c").read_text(
        encoding="utf-8"
    )
    main = (ROOT / "main.c").read_text(encoding="utf-8")
    oled = (ROOT / "Code" / "myOLED.c").read_text(encoding="utf-8")
    syscfg = (ROOT / "mspm0-modules.syscfg").read_text(encoding="utf-8")

    assert "MOTOR_SPEED_PULSES_PER_REV      13" in header
    assert "MOTOR_SPEED_GEAR_RATIO          20" in header
    assert "MOTOR_SPEED_WHEEL_CIRCUMFERENCE_M   (0.1508f)" in header
    assert "MOTOR_SPEED_WHEEL_CIRCUMFERENCE_M /" in source
    assert "MOTOR_SPEED_PULSES_PER_WHEEL * dt_s" in source
    assert "motor_speed_A_mps * 0.2f" in source
    assert "motor_speed_B_mps * 0.2f" in source

    assert 'GPIO5.associatedPins[0].pin.$assign  = "PA26"' in syscfg
    assert 'GPIO5.associatedPins[1].pin.$assign  = "PA25"' in syscfg
    assert 'GPIO7.associatedPins[0].pin.$assign  = "PA27"' in syscfg
    assert 'GPIO8.associatedPins[0].pin.$assign  = "PA14"' in syscfg
    assert "MotorSpeed_OnPulseA();" in interrupts
    assert "MotorSpeed_OnPulseB();" in interrupts
    assert "MotorSpeed_Update(0.01f);" in main
    assert "motor_speed_A_mps" in oled
    assert "motor_speed_B_mps" in oled


if __name__ == "__main__":
    test_encoder_speed_driver_matches_new_board()
    print("Encoder speed port invariants: OK")
