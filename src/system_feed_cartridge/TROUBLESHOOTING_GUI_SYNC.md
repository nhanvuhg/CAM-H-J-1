# Cartridge System GUI Mode Synchronization Troubleshooting

## The Problem
When pressing `STOP` on the GUI, the target object was to reset the operation mode so the operator would be required to explicitly re-select either `AUTO` or `MANUAL`. However, resetting `self.operation_mode = ''` in Python caused a dual-layered desynchronization bug:
1. **The C++ Level Desync**: The C++ logical controller (`current_mode_`) only logged mode changes on user clicks. So when Python reset the mode to `''`, C++ still retained `"manual"`, displaying a false status to the user.
2. **The QML String Parsing Lock**: To fix the C++ sync, Python's mode was appended to `/system_state` as `f"{self.state.value}|...|{self.operation_mode}"` resulting in a string like `"idle|idle|idle|manual"`. However, the QML UI (`CartridgePage.qml`) was strictly programmed to check `if (systemState === "idle")`. Providing it the piped string `"idle|idle|idle"` caused `systemState === "idle"` to evaluate to `false`, which completely locked the Mode Dropdown menu and JOG buttons, simulating a "system busy" state eternally.

## The Solution
To establish a robust two-way sync between Python's active state and the C++ UI without breaking QML's strict string expectations:
1. **Python Publisher**: Python appends `self.operation_mode` into the `/system_state` payload (e.g. `"idle|idle|idle|manual"`).
2. **C++ Interception**: The `CartridgeController` C++ subscription intercepts this string. It parses `parts[3]` to explicitly override `current_mode_` unconditionally, ensuring C++ and QML track Python's forced resets perfectly.
3. **C++ String Slicing**: Crucially, instead of forwarding the raw piped string to QML (which breaks the `=== "idle"` checks), C++ isolates `parts[0]` (the primary Cartesian state, e.g., `"idle"`) and assigns ONLY that to `system_state_`.

## Key Takeaway for Future AI Agents
When extending ROS 2 topics that cross the Python $\leftrightarrow$ C++ $\leftrightarrow$ QML boundary:
- Never assume QML parses substrings. If QML checks for `"idle"`, it expects exactly `"idle"`.
- If you need to ferry hidden state data from Python to C++, intercept and strip that data in the C++ layer BEFORE emitting the state property up to the strictly-typed QML UI layer.
- Mode synchronization must rely on the Python backend as the absolute Source of Truth via continuous polling payloads, rather than trusting the one-way event triggers from the C++ GUI.
