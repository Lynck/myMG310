from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_absent_vl53_is_not_accessed_at_runtime():
    main = (ROOT / "main.c").read_text(encoding="utf-8")

    assert "LeaderDistance_Init();" not in main
    assert "LeaderDistance_Process();" not in main
    assert "OLED_Init();" in main


if __name__ == "__main__":
    test_absent_vl53_is_not_accessed_at_runtime()
    print("OLED-only startup invariants: OK")
