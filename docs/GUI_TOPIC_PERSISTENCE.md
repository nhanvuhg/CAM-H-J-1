# GUI Auto-Restore Camera Topics - Implementation Summary

## Problem
When GUI crashes and restarts, users had to manually re-select camera topics (e.g., `/cam0HP/image_overlay` instead of `/cam0HP/image_raw`), which is inconvenient for production use.

## Solution: Topic Persistence

### Changes Made

#### 1. Added Persistence to `CamNode` Class

**File**: [cam_node.hpp](file:///home/pi/ros2_ws/src/ros2_qml_gui1/include/ros2_qml_gui/cam_node.hpp)
- Added methods: `saveTopicSelections()`, `loadTopicSelections()`
- Added private helpers: `saveTopicsToFile()`, `loadTopicsFromFile()`
- Added config file path: `configFilePath_`

**File**: [cam_node.cpp](file:///home/pi/ros2_ws/src/ros2_qml_gui1/src/cam_node.cpp)
- Constructor initializes config path: `~/.config/ros2_gui/camera_topics.conf`
- `saveTopicsToFile()`: Writes current topics to config file
- `loadTopicsFromFile()`: Reads topics from config file
- `updateCameraTopic()`: Auto-saves after every topic change
- `loadTopicSelections()`: Loads saved topics on startup, or auto-discovers if no config

#### 2. Updated GUI Initialization

**File**: [main.cpp](file:///home/pi/ros2_ws/src/ros2_qml_gui1/src/main.cpp)
- Replaced manual topic setup with `camNode->loadTopicSelections()`
- GUI now auto-restores last selected topics on startup

### How It Works

```
┌─────────────────────────────────────────────────┐
│  GUI Startup                                    │
├─────────────────────────────────────────────────┤
│  1. loadTopicSelections()                       │
│  2. Check ~/.config/ros2_gui/camera_topics.conf│
│  3. If exists → Load saved topics               │
│  4. If not exists → Auto-discover topics        │
│  5. Subscribe to topics                         │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│  User Changes Topic (via GUI dropdown)          │
├─────────────────────────────────────────────────┤
│  1. updateCameraTopic(index, newTopic)          │
│  2. Update subscription                         │
│  3. Auto-save to config file                    │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│  GUI Crashes & Restarts                         │
├─────────────────────────────────────────────────┤
│  1. loadTopicSelections() reads config          │
│  2. Restores exact same topics                  │
│  3. ✅ User sees correct camera feeds           │
└─────────────────────────────────────────────────┘
```

## Verification

### Test 1: First Run (No Config)
```bash
$ rm ~/.config/ros2_gui/camera_topics.conf
$ ros2 run ros2_qml_gui1 ros2_qml_gui1
[INFO] Config file not found (first run?), will use defaults
# GUI auto-discovers: /cam0HP/image_raw, /cam1HP/image_raw
```

### Test 2: Manual Config + Restart
```bash
$ cat ~/.config/ros2_gui/camera_topics.conf
/cam0HP/image_overlay
/cam1HP/image_overlay

$ ros2 run ros2_qml_gui1 ros2_qml_gui1
[INFO] Loaded 2 camera topics from /home/pi/.config/ros2_gui/camera_topics.conf
[INFO] Restored camera topics from config
# ✅ GUI subscribes to overlay topics as saved
```

### Test 3: Topic Change Auto-Save
```
User selects /cam0HP/image_overlay in GUI dropdown
→ updateCameraTopic(0, "/cam0HP/image_overlay") called
→ Auto-saves to config file
→ On next restart, overlay topic is restored ✅
```

## Files Changed

| File | Changes |
|------|---------|
| `cam_node.hpp` | Added persistence methods & config path |
| `cam_node.cpp` | Implemented save/load functions |
| `main.cpp` | Use loadTopicSelections() instead of hardcoded topics |

## Benefits for Production

✅ **No manual reconfiguration** after GUI crashes  
✅ **Automatic save** whenever user changes topics  
✅ **Fallback to auto-discovery** if config missing  
✅ **Persistent across reboots**  
✅ **User-specific** config file (~/.config)

## Usage

Users don't need to do anything special - the system automatically:
1. Saves their topic selections when they change them in GUI
2. Restores those selections when GUI restarts
3. Works seamlessly with the auto-restart wrapper script
