# Robot Pipeline Test Guide

## Pipeline State Machine Flow (AUTO Mode)

```
IDLE → INIT_CHECK → INIT_LOAD_CHAMBER_DIRECT → INIT_REFILL_BUFFER → WAIT_FILLING
                                                                         ↓
              ┌────────────────────────────────────────── fill_done ──────┘
              ↓
      TAKE_CHAMBER_TO_SCALE ──→ LOAD_CHAMBER_FROM_BUFFER ──→ REFILL_BUFFER
              │                        (buffer→chamber)       (input→buffer)
              │                                                    │
              └── (if buffer empty) ──→ PROCESSING_SCALE           │
                                           ↓                       ↓
PLACE_TO_OUTPUT  ←── scale PASS ───────────┘               WAIT_FILLING
       ↓                                                       (next cycle)
  WAIT_FILLING  (continue) ──or── IDLE (pipeline drained)
```

## External Signals Required Per Cycle

| State                | Signal Needed                                           | Topic                                              |
|---------------------|---------------------------------------------------------|----------------------------------------------------|
| IDLE → INIT_CHECK   | `start_system` service                                  | `/robot/start_system` (SetBool, data=true)         |
| INIT_LOAD (1st batch)| `feed_chamber`                                         | `/revpi/feed_chamber` (Bool, data=true)            |
| INIT_LOAD           | `new_tray_loaded`                                       | `/cartridge_providesystem/new_tray_loaded` (Bool)  |
| INIT_LOAD           | `command_row` (which row to pick)                       | `/robot/command_row` (Int32, 1-5)                  |
| **WAIT_FILLING**    | **`fill_done`**                                         | `/fill_machine/fill_done` (Bool, data=true)        |
| **PROCESSING_SCALE**| **`scale/result`** (PASS=true, FAIL=false)              | `/scale/result` (Bool)                             |
| **PLACE_TO_OUTPUT** | **`command_slot`** (which output slot)                  | `/robot/command_slot` (Int32, 1-9)                 |
| **PLACE_TO_OUTPUT** | **`new_trayoutput_loaded`** (output tray ready)         | `/cartridge_providesystem/new_trayoutput_loaded`   |

## Tray Change Flow

Input tray change triggers when `current_auto_row_` wraps from 5→1.
The system publishes `done_tray_input` and sets `waiting_for_tray_change_=true`.

To resume after tray change, send:
1. `/cartridge_providesystem/new_tray_loaded` = true
2. `/revpi/feed_chamber` = true
3. `/robot/command_row` = 1

## 7-Row Test Scenario

Start from row 4, 2 trays:
- **Tray 1**: Row 4, Row 5 → `done_tray_input` fires
- **Tray 2**: Row 1,2,3,4,5 → send `is_last_tray` before row 5 → pipeline drains → IDLE

### Setup Commands
```bash
# 1. Set AUTO mode
ros2 topic pub /robot/set_mode std_msgs/msg/Int32 "{data: 1}" --once

# 2. Set starting row
ros2 topic pub /robot/command_row std_msgs/msg/Int32 "{data: 4}" --once

# 3. Start system (blocks during HOME motion ~5s)
ros2 service call /robot/start_system std_srvs/srv/SetBool "{data: true}"

# 4. Signal new tray + feed chamber
ros2 topic pub /cartridge_providesystem/new_tray_loaded std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /revpi/feed_chamber std_msgs/msg/Bool "{data: true}" --once

# 5. Signal output tray ready
ros2 topic pub /cartridge_providesystem/new_trayoutput_loaded std_msgs/msg/Bool "{data: true}" --once
```

### Driver Script
Use `/tmp/test_7rows.py` — it subscribes to `/robot/system_status` and dispatches signals at each state.

> **IMPORTANT**: `/robot/system_status` uses dedup (only publishes on state change).
> A new subscriber will NOT receive the current state if no transition is happening.
> The driver must handle empty initial status by sending signals for multiple states.

## Key Gotchas

1. **publishSystemStatus dedup**: New subscribers miss current state. Must handle `status=""`.
2. **PLACE_TO_OUTPUT needs 2 signals**: Both `command_slot` AND `new_trayoutput_loaded`.
3. **INIT_LOAD first-batch**: Requires `feed_chamber_signal_` only on `is_first_batch_`.
4. **Row 5→1 wrap**: Triggers `done_tray_input` automatically. Pipeline blocks until new tray signals.
5. **Slot 9**: Triggers `done_tray_output` + output tray change + system goes to IDLE.
6. **is_last_batch**: Set when `is_last_tray_available_ && current_auto_row_` wraps. Pipeline drains all positions then goes IDLE.
