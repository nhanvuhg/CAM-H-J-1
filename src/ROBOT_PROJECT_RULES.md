---
description: Critical Rules for AI Agents Editing the Dobot/Cartridge ROS2 Workspace
---

# 🤖 ROS2 & ROBOT PROJECT RULES FOR AI AGENTS
**CRITICAL:** Any AI Agent interacting with this workspace MUST read and adhere to these rules before implementing code changes. Failure to do so will result in severe system desynchronization or operational hangs.

## Rule 1: Always Recompile ROS2 Packages (Even Python!)
In this project environment, Python nodes (like `cartridge_providesystem_py_node.py`) are copied to the `install/` directory by `colcon`. They do **NOT** use `symlink-install` by default.
- **Do not assume** that saving a `.py` file in `src/` automatically applies it to the runtime.
- **MANDATORY ACTION:** After modifying any file (C++ or Python) in a ROS2 package, you **MUST** run:
  ```bash
  cd /home/pi/ros2_ws
  colcon build --packages-select <package_name>
  ```
- **MANDATORY ACTION:** After building, the terminal service must be restarted (e.g., `systemctl restart system_feed_cartridge` or `robot_control`).

## Rule 2: Manual Overrides Must Preserve the Pipeline Mode
When modifying GUI callbacks or override commands (like `PLACE_TO_OUTPUT` or `START`):
- Do not blindly force the system into `manual_mode_`. If a user triggers a fallback action to rescue a stalled `AUTO` pipeline, ensure the node completes the rescue action and safely continues the Auto sequence.
- Always check what flags block the state machine. Examples:
  - Is it waiting for an `ignore_scale` timeout?
  - Does the override command skip the AI processing, leaving slot variables like `selected_output_slot_` as `SLOT_UNSET` (-1)?
- Provide automatic fallbacks (like `current_auto_slot_`) to rescue stalled states gracefully.

## Rule 3: Maintain State Machine Homing & Interlocks
- **Homing Resets:** If modifying `STOP` or `E-STOP` logic, ensure homing offset variables (`self.zero_offset.clear()`) are properly reset so that the subsequent `START` command forces a fresh homing sequence.
- **Sensor Index Mapping:** Hardware sensor names have been re-mapped (e.g., Legacy `s15/s16` are now `s13/s14`). When resolving NameErrors or sensor logic loops, you **MUST** cross-reference the active mapping table in `sensor_mapping.md`. Do not guess sensor indexes.

## Rule 4: System Verifications
Before declaring any task complete:
1. Verify the code compiles without syntax errors (use `python3 -m py_compile` for python).
2. Verify you have executed `colcon build` to sync the `install/` folder.
3. Explicitly document the manual command needed for the user to restart the associated demon/service.

## Rule 5: Strict Adherence to System Logic & Architecture Graphs
- **DO NOT ALTER CORE LOGIC:** The established state machine architecture, parallel sequences, and system workflow graphs are fixed. You must strictly follow the existing logic flows.
- Do not refactor, bypass, or redesign core pipelines (such as converting asynchronous tasks into synchronous loops) unless the user explicitly commands a structural change.
- If a bug is found within the state machine, apply localized minimal fixes that respect and preserve the original design intent and architectural graphs.
