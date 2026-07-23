"""任务一、任务二丢线速度必须保持相互独立。"""

from pathlib import Path


source = (Path(__file__).parents[1] / "Code" / "myTask.c").read_text(
    encoding="utf-8"
)

assert "#define TASK1_LINE_LOST_SPEED        16" in source
assert "current_task == TASK_ID_1" in source
assert "return TASK1_LINE_LOST_SPEED;" in source
assert "return controlled_base_speed;" in source
assert "-line_lost_speed, line_lost_speed" in source
assert "line_lost_speed, line_lost_speed" in source

print("Task-specific line-lost speed invariants: OK")
