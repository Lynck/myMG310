from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_pb21_run_button_configuration_and_control_path():
    syscfg = (ROOT / "mspm0-modules.syscfg").read_text(encoding="utf-8")
    main = (ROOT / "main.c").read_text(encoding="utf-8")
    module = (ROOT / "Code" / "run_button.c").read_text(encoding="utf-8")

    assert 'GPIO9.$name                              = "RUN_BUTTON"' in syscfg
    assert 'GPIO9.associatedPins[0].internalResistor = "PULL_UP"' in syscfg
    assert 'GPIO9.associatedPins[0].pin.$assign      = "PB21"' in syscfg

    timer_path = main[main.index("if (timer_10ms_flag)") :]
    assert "RunButton_Update();" in timer_path
    assert timer_path.index("RunButton_Update();") < timer_path.index(
        "Bluetooth_CheckSyncTimeout();"
    )

    stop_path = module[module.index("if (g_line_follow_enabled)") :]
    assert "g_line_follow_enabled = false;" in stop_path
    assert "Motor_Brake();" in stop_path


if __name__ == "__main__":
    test_pb21_run_button_configuration_and_control_path()
    print("PB21 run-button integration invariants: OK")
