// ─────────────────────────────────────────────────────────────────────────────
// FillHpTab.qml — replacement cho PAGE 4 (FILL HP CONTROL) trong CartridgePage.
// Read-only display + cac control AN TOAN (Start/Stop/Mode/Reconnect/Clear Error).
// CO Y BO QUA cac control rui ro cao: manual valve/cylinder, settings editor,
// servo jog. Khi can them lai, mo file nay, KHONG dung sang CartridgePage.qml.
// ─────────────────────────────────────────────────────────────────────────────
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: tab
    property Item focusHost: null

    // ---- Theme tokens (đồng bộ liquid-glass với CameraPage/CartridgePage/InkTab) ----
    readonly property color cBg:          "transparent"
    readonly property color cPanel:       "#990d1e32"
    readonly property color cPanel2:      "#8806101d"
    readonly property color cBorder:      "#1affffff"
    readonly property color cText:        "#ffffff"
    readonly property color cMuted:       "#bfe0f5"
    readonly property color cAccent:      "#67d0ff"
    readonly property color cAccentSoft:  Qt.rgba(0.42, 0.75, 0.95, 0.16)
    readonly property color cOk:          "#3ed0b4"
    readonly property color cOkBg:        Qt.rgba(0.0, 0.90, 0.46, 0.15)
    readonly property color cField:       "#cc081627"
    readonly property color cWarn:        "#f5a623"
    readonly property color cWarnBg:      Qt.rgba(1.0, 0.65, 0.15, 0.15)
    readonly property color cBad:         "#f0735c"
    readonly property color cBadBg:       Qt.rgba(1.0, 0.32, 0.32, 0.15)
    readonly property color cIdle:        "#74899f"
    readonly property color cIdleBg:      Qt.rgba(0.32, 0.38, 0.44, 0.15)
    readonly property color cBtnBaseStart:"#0c1726"
    readonly property color cBtnBaseEnd:  "#06101d"
    readonly property color cBtnPrimaryStart:"#1f9e86"
    readonly property color cBtnPrimaryEnd:  "#163a52"
    readonly property color cBtnActionStart: "#1a4a6e"
    readonly property color cBtnActionEnd:   "#0c1726"
    readonly property color cBtnActionHoverStart: "#1a4a6e"
    readonly property color cBtnActionHoverEnd:   "#163a52"
    readonly property color cBtnActionPressStart: "#163a52"
    readonly property color cBtnActionPressEnd:   "#04080f"
    readonly property color cBtnWarnStart:   "#8a4210"
    readonly property color cBtnWarnEnd:     "#E68457"
    readonly property color cBtnDangerStart: "#E05454"
    readonly property color cBtnDangerEnd:   "#7a2424"
    readonly property color cControlPanel:   "#990d1e32"
    readonly property color cControlBorder:  "#1affffff"
    readonly property color cControlHover:   "#40ffffff"
    readonly property color cSensorIdleBg:   Qt.rgba(0.03, 0.11, 0.18, 0.18)
    readonly property color cSensorIdleBorder: Qt.rgba(0.08, 0.22, 0.32, 0.42)
    readonly property color cSensorIdleText: Qt.rgba(0.62, 0.70, 0.78, 0.55)
    readonly property color cSensorIdleDot:  Qt.rgba(0.08, 0.22, 0.32, 0.34)
    readonly property color cSensorActiveStart: "#CAE8D5"
    readonly property color cSensorActiveEnd:   "#163a52"
    readonly property color cSensorActiveBorder:"#163a52"
    readonly property color cSensorActiveText:  "#06101d"
    readonly property color cIoActiveStart:     cBtnActionStart
    readonly property color cIoActiveEnd:       cBtnActionEnd
    readonly property color cIoActiveText:      "#ffffff"

    readonly property string monoFamily:  "monospace"
    readonly property string dashboardTextFamily: Qt.application.font.family

    // Match Control Dashboard typography. Numeric fields can override this
    // with monoFamily, exactly as they do on the dashboard.
    component DashboardText: Text {
        font.family: tab.dashboardTextFamily
    }

    // ---- Parsers (mirror page4Root logic) ----
    function parseKvPipe(raw) {
        var out = {};
        if (!raw) return out;
        var parts = String(raw).split("|");
        for (var i = 0; i < parts.length; i++) {
            var part = parts[i];
            var idx = part.indexOf('=');
            var alt = part.indexOf(':');
            var pos = idx >= 0 ? idx : alt;
            if (pos >= 0) out[part.substring(0, pos).trim()] = part.substring(pos + 1).trim();
        }
        return out;
    }
    function parseKvComma(raw) {
        var out = {};
        if (!raw) return out;
        var parts = String(raw).split(",");
        for (var i = 0; i < parts.length; i++) {
            var part = parts[i];
            var idx = part.indexOf('=');
            var alt = part.indexOf(':');
            var pos = idx >= 0 ? idx : alt;
            if (pos >= 0) {
                var k = part.substring(0, pos).trim();
                var v = part.substring(pos + 1).trim();
                if (k === "safety_ch1") {
                    k = "safety_i_4_i04";
                } else if (k === "safety_ch2") {
                    k = "safety_i_5_i04";
                }
                out[k] = v;
            }
        }
        return out;
    }
    function classifyState(rawVal) {
        var v = String(rawVal || "").toLowerCase();
        if (v === "on" || v === "true" || v === "ok" || v === "ready" || v === "connected") return "on";
        if (v === "off" || v === "false" || v === "error" || v === "fault" || v === "disconnected") return "off";
        return "mid";
    }
    function getSensorLabel(key) {
        var mapping = {
            "start_button": "Start button",
            "stop_button": "Stop button",
            "optical_sensor": "Optical sensor",
            "ball_feed_down": "Ball feed down",
            "ball_feed_up": "Ball feed up",
            "ball_push_down": "Ball push down",
            "ball_push_up": "Ball push up",
            "chamber_open": "Chamber open",
            "chamber_closed": "Chamber closed",
            "seal_pin_down": "Seal pin down",
            "seal_pin_up": "Seal pin up",
            "fix_cylinder_down": "Fix cyl down",
            "fix_cylinder_up": "Fix cyl up",
            "ball_box_empty": "Ball box empty",
            "safety_i_4_i04": "Safety I_4_i04",
            "safety_i_5_i04": "Safety I_5_i04",
            "safety_area_clear": "Safety area clear"
        };
        if (mapping[key] !== undefined) {
            return mapping[key];
        }
        if (key.indexOf("mag_") === 0) {
            return "Mag " + key.substring(4);
        }
        if (key.indexOf("tube_") === 0) {
            return "Tube " + key.substring(5);
        }
        return key;
    }
    function classifyPressure(val, lowT, highT, limitT) {
        if (val < lowT) return "low";
        if (val >= limitT) return "limit";
        if (val >= highT) return "high";
        return "ok";
    }
    function pressureFillStart(cls) {
        return "#8edcff";
    }
    function pressureFillEnd(cls) {
        return "#0a3d56";
    }
    function pressureTrackColor(cls) {
        return "#071421";
    }
    function pressureTextColor(cls) {
        return "#8edcff";
    }
    function getCartridgeStats() {
        var list = hpController.cartridgePressures;
        if (!list || list.length === 0) return "Min 0.0   Avg 0.0   Max\n0.0 mbar";
        var minVal = Number.MAX_VALUE;
        var maxVal = -Number.MAX_VALUE;
        var sum = 0.0;
        var count = 0;
        for (var i = 0; i < list.length; i++) {
            var v = parseFloat(list[i]);
            if (!isNaN(v)) {
                if (v < minVal) minVal = v;
                if (v > maxVal) maxVal = v;
                sum += v;
                count++;
            }
        }
        if (count === 0) return "Min 0.0   Avg 0.0   Max\n0.0 mbar";
        var avgVal = sum / count;
        return "Min " + minVal.toFixed(1) + "   Avg " + avgVal.toFixed(1) + "   Max\n" + maxVal.toFixed(1) + " mbar";
    }
    // Parse valveState format "v1=open:on,v2=ink:on,..." -> { v1: {label, state} }
    function parseValveState(rawStr) {
        var out = {};
        if (!rawStr) return out;
        var parts = String(rawStr).split(",");
        for (var i = 0; i < parts.length; i++) {
            var p = parts[i].split("=");
            if (p.length === 2) {
                var chunks = p[1].trim().split(":");
                out[p[0].trim()] = { label: chunks[0] || "-", state: chunks[1] || "mid" };
            }
        }
        return out;
    }
    // Parse alert string format "LEVEL:WARN|TIME:08:52:03|AREA:CHAMBER_SAFETY|MESSAGE:..."
    function parseAlertString(rawStr) {
        var out = { level: "", time: "", area: "", message: "", sev: "info", raw: rawStr };
        if (!rawStr) return out;
        var parts = String(rawStr).split("|");
        for (var i = 0; i < parts.length; i++) {
            var p = parts[i];
            var idx = p.indexOf(':');
            if (idx < 0) continue;
            var key = p.substring(0, idx).trim().toUpperCase();
            var val = p.substring(idx + 1).trim();
            if (key === "LEVEL") out.level = val;
            else if (key === "TIME") out.time = val;
            else if (key === "AREA") out.area = val;
            else if (key === "MESSAGE") out.message = val;
        }
        // Classify severity
        var lvl = out.level.toUpperCase();
        var msg = (out.message + " " + out.area).toLowerCase();
        if (lvl === "CRITICAL" || msg.indexOf("critical") >= 0 || msg.indexOf("emergency") >= 0) out.sev = "critical";
        else if (lvl === "ERROR" || lvl === "FAULT" || msg.indexOf("error") >= 0 || msg.indexOf("fault") >= 0 || msg.indexOf("timeout") >= 0) out.sev = "error";
        else if (lvl === "WARN" || lvl === "WARNING" || msg.indexOf("warn") >= 0) out.sev = "warning";
        else out.sev = "info";
        if (!out.message) out.message = rawStr;
        return out;
    }

    // Action label matching with aliases (open/close/on/off/air/ink/up/down/chamber/waste)
    function actionMatches(label, action) {
        if (!label || !action) return false;
        var text = String(label).toLowerCase().trim();
        var act  = String(action).toLowerCase().trim();
        var aliases = {
            "open":["open","mở"], "close":["close","đóng"],
            "on":["on","bật"], "off":["off","tắt"],
            "air":["air"], "ink":["ink"], "chamber":["chamber"], "waste":["waste"],
            "up":["up","lên"], "down":["down","xuống"]
        };
        var list = aliases[act] || [act];
        return list.indexOf(text) !== -1;
    }

    // ---- Derived data from hpController ----
    property var sysMap:      parseKvPipe(hpController.systemStatus)
    property var hwMap:       parseKvPipe(hpController.hwStatus)
    property var inputsMap:   parseKvComma(hpController.inputState)
    property var inkMap:      parseKvPipe(hpController.inkStatus)

    // ---- Alert center state (accumulate alerts client-side) ----
    property var alertHistory: []          // [{level, time, area, message, sev, raw}, ...]
    property string lastAlertRaw: ""
    property string alertFilter: "all"     // all | critical | error | warning | info
    readonly property var filteredAlerts: alertHistory.filter(function(a) {
        if (tab.alertFilter === "all") return true;
        return a.sev === tab.alertFilter;
    })
    readonly property int countError:    alertHistory.filter(function(a) { return a.sev === "error" || a.sev === "critical" }).length
    readonly property int countWarning:  alertHistory.filter(function(a) { return a.sev === "warning" }).length
    readonly property int countInfo:     alertHistory.filter(function(a) { return a.sev === "info" }).length

    Connections {
        target: hpController
        function onErrorStatusChanged() {
            var raw = hpController.errorStatus;
            if (!raw || raw === "OK" || raw === "-" || raw === tab.lastAlertRaw) return;
            tab.lastAlertRaw = raw;
            var parsed = tab.parseAlertString(raw);
            var hist = tab.alertHistory.slice();
            hist.unshift(parsed);
            if (hist.length > 50) hist = hist.slice(0, 50);
            tab.alertHistory = hist;
        }
        function onManualResponseChanged() {
            var msg = hpController.manualResponse;
            if (!msg || msg === "-" || msg === tab.lastActionRaw) return;
            tab.lastActionRaw = msg;
            var now = new Date();
            var hh = String(now.getHours()).padStart(2, '0');
            var mm = String(now.getMinutes()).padStart(2, '0');
            var ss = String(now.getSeconds()).padStart(2, '0');
            var line = hh + ":" + mm + ":" + ss + " · " + msg;
            var log = tab.actionLog.slice();
            log.unshift(line);
            if (log.length > 30) log = log.slice(0, 30);
            tab.actionLog = log;
        }
    }

    // ---- Action log state ----
    property var actionLog: []
    property string lastActionRaw: ""
    // Lay danh sach key inputs, loc bo mag_index va tube_index de lay 33 binary sensors
    property var filteredInputKeys: Object.keys(inputsMap).filter(function(k) {
        return k !== "mag_index" && k !== "tube_index"
    })
    property var valvesMap:   parseValveState(hpController.valveState)
    property var settingsMap: parseKvComma(hpController.pressureThresholds)
    property int activeSettingsTab: 0

    // ---- Static models ported from PAGE 4 (DO NOT change id/statusKey/a/b
    //      — must match hp_controller publishManual topic format) ----
    property var valveModel: [
        { id: "valve1",        statusKey: "v1",      label: "V1 (Main)",       a: "open",    b: "close" },
        { id: "valve2",        statusKey: "v2",      label: "V2 (Air/Ink)",    a: "air",     b: "ink" },
        { id: "valve4",        statusKey: "v4",      label: "V4 (Main Air)",   a: "on",      b: "off" },
        { id: "valve5",        statusKey: "v5",      label: "V5 (Tank ret)",   a: "open",    b: "close" },
        { id: "valve6",        statusKey: "v6",      label: "V6 (Clean)",      a: "open",    b: "close" },
        { id: "valve7",        statusKey: "v7",      label: "V7 (Purge Ink)",  a: "on",      b: "off" },
        { id: "valve8",        statusKey: "v8",      label: "V8 (Purge Air)",  a: "on",      b: "off" },
        { id: "valve9",        statusKey: "v9",      label: "V9 (Waste/Ch)",   a: "chamber", b: "waste" },
        { id: "valve_chamber", statusKey: "vchamber",label: "V Chamber",       a: "on",      b: "off" },
        { id: "vacuum",        statusKey: "pump",    label: "Vacuum Pump",     a: "on",      b: "off" }
    ]
    property var cylinderModel: [
        { id: "chamber",   statusKey: "cyl_ch",   label: "Chamber Cyl",   a: "close", b: "open" },
        { id: "cartridge", statusKey: "cyl_cart", label: "Cartridge Cyl", a: "down",  b: "up" },
        { id: "ball_feed", statusKey: "cyl_bf",   label: "Ball Feed Cyl", a: "up",    b: "down" },
        { id: "ball_push", statusKey: "cyl_bp",   label: "Ball Push Cyl", a: "up",    b: "down" },
        { id: "seal_pin",  statusKey: "cyl_seal", label: "Seal Pin Cyl",  a: "up",    b: "down" },
        { id: "fix_cyl",   statusKey: "cyl_fix",  label: "Fix Cylinder",  a: "up",    b: "down" }
    ]
    property var settingGroups: [
        { id: "auto", label: "Auto", items: [
            { topic: "cart_fix",          type: "threshold", label: "Cart Fix Pressure", min: 0,    max: 1200, unit: "mbar" },
            { topic: "chamber_vac",       type: "threshold", label: "Chamber Vacuum",    min: 0,    max: 1200, unit: "mbar" },
            { topic: "chamber_leak",      type: "threshold", label: "Chamber Leak",      min: 0,    max: 1200, unit: "mbar" },
            { topic: "cart_vac",          type: "threshold", label: "Cartridge Vacuum",  min: 0,    max: 1200, unit: "mbar" },
            { topic: "cart_leak",         type: "threshold", label: "Cartridge Leak",    min: 0,    max: 1200, unit: "mbar" },
            { topic: "pressure_balance",  type: "threshold", label: "Pressure Balance",  min: 0,    max: 1200, unit: "mbar" },
            { topic: "waste_ink_drain_duration", type: "float", label: "Waste Ink Drain Time", min: 0, max: 120, unit: "s" },
            { topic: "chamber_vent",      type: "threshold", label: "Chamber Vent",      min: 0,    max: 1200, unit: "mbar" },
            { topic: "fill_compensation", type: "float",     label: "Fill Compensation", min: -999, max: 999,  unit: "ml" }
        ]},
        { id: "clean", label: "Clean", items: [
            { topic: "cr_cart_vac",         type: "threshold", label: "Cartridge Vacuum", min: 0, max: 1200, unit: "mbar" },
            { topic: "cr_cart_leak",        type: "threshold", label: "Cartridge Leak",   min: 0, max: 1200, unit: "mbar" },
            { topic: "cr_pressure_balance", type: "threshold", label: "Pressure Balance", min: 0, max: 1200, unit: "mbar" },
            { topic: "cr_volume",           type: "float",     label: "CR Volume",        min: 0, max: 9999, unit: "ml" },
            { topic: "cr_flow_rate",        type: "float",     label: "CR Flow",          min: 0, max: 100,  unit: "ml/s" },
            { topic: "cr_loading_rate",     type: "float",     label: "CR Loading",       min: 0, max: 100,  unit: "ml/s" },
            { topic: "cr_cycles",           type: "int",       label: "CR Cycles",        min: 1, max: 999,  unit: "" }
        ]},
        { id: "dosing", label: "Dosing", items: [
            { topic: "dosing_volume",       type: "float", label: "Dosing Volume",  min: 0, max: 9999, unit: "ml" },
            { topic: "dosing_flow_rate",    type: "float", label: "Dosing Flow",    min: 0, max: 100,  unit: "ml/s" },
            { topic: "dosing_loading_rate", type: "float", label: "Dosing Loading", min: 0, max: 100,  unit: "ml/s" }
        ]},
        { id: "pwm", label: "PWM/Tank", items: [
            { topic: "base_pwm",            type: "int",   label: "Base PWM",         min: 0,   max: 100,  unit: "%" },
            { topic: "chamber_vent_pwm",    type: "int",   label: "Chamber Vent PWM", min: 0,   max: 100,  unit: "%" },
            { topic: "cr_valve10_pwm",      type: "int",   label: "CR Valve10 PWM",   min: 0,   max: 100,  unit: "%" },
            { topic: "cr_valve10_duration", type: "float", label: "CR Valve10 Time",  min: 0.5, max: 30,   unit: "s" },
            { topic: "cr_valve5_duration",  type: "float", label: "CR Valve5 Time",   min: 0.5, max: 30,   unit: "s" },
            { topic: "cr_return_duration",  type: "float", label: "CR Return Time",   min: 0.5, max: 30,   unit: "s" },
            { topic: "tank_min",            type: "float", label: "Tank Min",         min: 0,   max: 1000, unit: "mbar" },
            { topic: "tank_max",            type: "float", label: "Tank Max",         min: 0,   max: 1000, unit: "mbar" }
        ]}
    ]
    // Helper: publish setting based on type (matches old PAGE 4 wire-up)
    function publishSetting(item, valStr) {
        var n = parseFloat(valStr);
        if (isNaN(n)) return;
        if (item.type === "threshold") hpController.publishString("pressure_thresholds_set", item.topic + ":" + valStr);
        else if (item.type === "int")  hpController.publishInt(item.topic, Math.round(n));
        else                            hpController.publishFloat(item.topic, n);
    }
    function settingCurrentValue(item) {
        if (!item || !item.topic) return "";
        if (item.topic === "base_pwm") return hpController.basePwmStatus.toString();
        return settingsMap[item.topic] || "";
    }
    function settingDisplayLabel(item) {
        return item && item.label ? String(item.label).toUpperCase() : "";
    }
    property string modeStr:   (sysMap["MODE"] || hpController.modeStatus || "-").toString().toUpperCase()
    property string stateStr:  sysMap["STATE"] || hpController.systemStatus || "-"
    property string cycleStr:  sysMap["CYCLE"] || "-"
    property string volumeStr: sysMap["VOLUME"] || "-"
    property bool   running:   String(sysMap["RUNNING"]).toLowerCase() === "true"
    property bool   hasError:  hpController.errorStatus && hpController.errorStatus !== "OK"
                              && hpController.errorStatus !== "-"
    function inkNameText() {
        return inkMap["NAME"] || inkMap["INK_NAME"] || inkMap["CODE"] || "NAME";
    }
    function inkCodeLotText() {
        var code = inkMap["CODE"] || "-";
        var lotPi = inkMap["LOT_PI"] || "-";
        var lotCi = inkMap["LOT_CI"] || "-";
        return code + " · Lot PI: " + lotPi + " · Lot CI: " + lotCi;
    }
    function safetyOk(key) {
        return classifyState(inputsMap[key]) === "on";
    }
    function processAutoText() {
        if (modeStr !== "AUTO") return "-";
        var m = stateStr.match(/FILL:(\d+)/);
        return "FILL " + (m ? m[1] : "-");
    }
    function processDosingText() {
        var m = stateStr.match(/DOSING:(\d+)/);
        if (m) return "DOSING " + m[1];
        return (modeStr === "CLEAN" || modeStr === "PREFILL") ? "CR dosing" : "-";
    }
    function processCleanText() {
        if (modeStr !== "CLEAN" && modeStr !== "PREFILL") return "-";
        var m = stateStr.match(/CR:(\d+)/);
        return "CR " + (m ? m[1] : "-");
    }

    Rectangle { anchors.fill: parent; color: cBg }

    // ====================================================================
    //  HEADER BAR (sticky, top)
    // ====================================================================
    Rectangle {
        id: headerBar
        anchors { top: parent.top; left: parent.left; right: parent.right }
        height: 60
        color: "#06101d"

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 14
            anchors.rightMargin: 14
            spacing: 8

            DashboardText {
                text: "Fill HP Control"
                color: cText; font.pixelSize: 28; font.bold: true
            }

            Item { Layout.fillWidth: true }

            TbBtn {
                lbl: "START"; variant: "primary"
                iconSource: "qrc:/qml/icons/play.svg"
                Layout.preferredWidth: 132
                Layout.preferredHeight: 40
                onClicked: hpController.publishScreenControl("start")
            }

            TbBtn {
                lbl: "STOP"; variant: "danger"
                iconSource: "qrc:/qml/icons/octagon_x_lucide.svg"
                Layout.preferredWidth: 132
                Layout.preferredHeight: 40
                onClicked: hpController.publishScreenControl("stop")
            }

            Item { width: 8 }

            TbBtn {
                lbl: "Clear Error"; variant: "warn"
                visible: tab.hasError
                onClicked: hpController.publishString("error_control", "clear")
            }
        }

        Rectangle {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            height: 1
            color: cBorder
        }
    }

    // ====================================================================
    //  BODY (scrollable)
    // ====================================================================
    Flickable {
        id: bodyScrollView
        anchors { top: headerBar.bottom; left: parent.left; right: parent.right; bottom: parent.bottom }
        contentWidth: width
        contentHeight: mainCol.implicitHeight + 24
        clip: true
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }
        onMovementEnded: {
            var targetY = Math.max(0, manualPage.y + mainCol.y - 12);
            if (contentY > height * 0.35 && targetY > 0)
                pageSnapAnim.to = Math.max(0, Math.min(targetY, contentHeight - height));
            else
                pageSnapAnim.to = 0;
            pageSnapAnim.restart();
        }

        NumberAnimation {
            id: pageSnapAnim
            target: bodyScrollView
            property: "contentY"
            duration: 280
            easing.type: Easing.OutCubic
        }

        ColumnLayout {
            id: mainCol
            x: 12; y: 12
            width: parent.width - 24
            spacing: 12

            // -- Error banner --
            Sect {
                Layout.fillWidth: true
                visible: tab.hasError
                bgColor: Qt.rgba(1.0, 0.32, 0.32, 0.12)
                borderColor: cBad
                noTitle: true

                RowLayout {
                    spacing: 12
                    width: parent.width

                    DashboardText { text: "⛔"; font.pixelSize: 30 }
                    ColumnLayout {
                        Layout.fillWidth: true; spacing: 2
                        DashboardText { text: "SYSTEM WARNING"; color: cBad; font.bold: true; font.pixelSize: 22 }
                        DashboardText {
                            text: hpController.errorStatus || "-"
                            color: cText; font.pixelSize: 21
                            wrapMode: Text.WrapAtWordBoundaryOrAnywhere; Layout.fillWidth: true
                        }
                    }
                    TbBtn {
                        lbl: "Acknowledge & Clear"; variant: "danger"
                        onClicked: hpController.publishString("error_control", "clear")
                    }
                }
            }

            // -- PAGE 1: compact overview + safety/settings + grouped sensor signal --
            RowLayout {
                id: pageOneGroup
                property real targetHeight: Math.max(820, bodyScrollView.height - 16)
                Layout.fillWidth: true
                Layout.preferredHeight: targetHeight
                spacing: 12

                ColumnLayout {
                    id: leftPageStack
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignTop
                    spacing: 10

                    RowLayout {
                        id: overviewCardsRow
                        property real cardHeight: Math.max(modeSelectionPanel.implicitHeight,
                                                           topOverviewPanel.implicitHeight,
                                                           topSafetyPanel.implicitHeight)
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignTop
                        spacing: 12

                        ModeSelectionPanel {
                            id: modeSelectionPanel
                            Layout.fillWidth: true
                            Layout.preferredWidth: 240
                            Layout.preferredHeight: overviewCardsRow.cardHeight
                            Layout.alignment: Qt.AlignTop
                        }
                        SystemOverviewPanel {
                            id: topOverviewPanel
                            Layout.fillWidth: true
                            Layout.preferredWidth: 440
                            Layout.preferredHeight: overviewCardsRow.cardHeight
                            Layout.alignment: Qt.AlignTop
                        }
                        SafetyProcessPanel {
                            id: topSafetyPanel
                            Layout.fillWidth: true
                            Layout.preferredWidth: 340
                            Layout.preferredHeight: overviewCardsRow.cardHeight
                            Layout.alignment: Qt.AlignTop
                        }
                    }

                    OperationalSettingsPanel {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.preferredHeight: Math.max(implicitHeight, pageOneGroup.targetHeight - topOverviewPanel.implicitHeight - 10)
                        Layout.alignment: Qt.AlignLeft
                    }
                }

                ColumnLayout {
                    id: sensorStack
                    Layout.preferredWidth: 760
                    Layout.maximumWidth: 760
                    Layout.alignment: Qt.AlignTop
                    spacing: 10

                    SensorGroupCard {
                        title: "SENSOR"
                        columns: 8
                        squareTiles: true
                        items: [
                            "start_button",
                            "stop_button",
                            "optical_sensor",
                            "ball_feed_down",
                            "ball_feed_up",
                            "ball_push_down",
                            "ball_push_up",
                            "chamber_open",
                            "chamber_closed",
                            "seal_pin_down",
                            "seal_pin_up",
                            "fix_cylinder_down",
                            "fix_cylinder_up",
                            "ball_box_empty",
                            "safety_i_4_i04",
                            "safety_i_5_i04",
                            "safety_area_clear"
                        ]
                        Layout.fillWidth: true
                    }

                    SensorGroupCard {
                        title: "TUBE 1-8"
                        columns: 8
                        squareTiles: true
                        items: [
                            "tube_1",
                            "tube_2",
                            "tube_3",
                            "tube_4",
                            "tube_5",
                            "tube_6",
                            "tube_7",
                            "tube_8"
                        ]
                        Layout.fillWidth: true
                    }

                    SensorGroupCard {
                        title: "MAG 1-8"
                        columns: 8
                        squareTiles: true
                        items: [
                            "mag_1",
                            "mag_2",
                            "mag_3",
                            "mag_4",
                            "mag_5",
                            "mag_6",
                            "mag_7",
                            "mag_8"
                        ]
                        Layout.fillWidth: true
                    }
                }
            }

            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(0, bodyScrollView.height - pageOneGroup.targetHeight - 36)
            }

            ManualControlsPage {
                id: manualPage
                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(820, bodyScrollView.height - 24)
                Layout.topMargin: 18
            }
        }
    }

    // ====================================================================
    //  COMPONENTS
    // ====================================================================

    component ModeSelectionPanel: Item {
        implicitHeight: 286

        Sect {
            anchors.fill: parent
            title: "MODE SELECTION"
            fillBodyHeight: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 6

                ModeSelectionButton {
                    lbl: "AUTO"
                    desc: "Automatic"
                    active: tab.modeStr === "AUTO"
                    onClicked: hpController.publishMode(0)
                }
                ModeSelectionButton {
                    lbl: "CLEAN"
                    desc: "Cleaning cycle"
                    active: tab.modeStr === "CLEAN"
                    onClicked: hpController.publishMode(1)
                }
                ModeSelectionButton {
                    lbl: "MANUAL"
                    desc: "Direct control"
                    active: tab.modeStr === "MANUAL"
                    onClicked: hpController.publishMode(2)
                }
                ModeSelectionButton {
                    lbl: "REFILL"
                    desc: "Refill cycle"
                    active: tab.modeStr === "PREFILL" || tab.modeStr === "REFILL"
                    onClicked: hpController.publishMode(3)
                }
            }
        }
    }

    component ModeSelectionButton: Rectangle {
        id: modeCardButton
        property string lbl: ""
        property string desc: ""
        property bool active: false
        signal clicked()

        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumHeight: 48
        radius: 6
        color: "transparent"
        border.color: active ? cSensorActiveBorder : cBorder
        border.width: 1
        gradient: active ? selectedModeGradient : idleModeGradient

        Gradient {
            id: selectedModeGradient
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: cSensorActiveStart }
            GradientStop { position: 1.0; color: cSensorActiveEnd }
        }
        Gradient {
            id: idleModeGradient
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: cBtnBaseStart }
            GradientStop { position: 1.0; color: cBtnBaseEnd }
        }

        Column {
            anchors.centerIn: parent
            spacing: 1
            DashboardText {
                anchors.horizontalCenter: parent.horizontalCenter
                text: modeCardButton.lbl
                color: modeCardButton.active ? cSensorActiveText : cText
                font.pixelSize: 14
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
            }
            DashboardText {
                anchors.horizontalCenter: parent.horizontalCenter
                text: modeCardButton.desc
                color: modeCardButton.active ? cSensorActiveText : cMuted
                opacity: modeCardButton.active ? 0.8 : 0.65
                font.pixelSize: 9
                horizontalAlignment: Text.AlignHCenter
            }
        }

        MotionMouseArea {
            anchors.fill: parent
            cursorShape: Qt.PointingHandCursor
            hoverScale: 1.012
            pressScale: 0.99
            shadowEnabled: false
            shimmerEnabled: false
            onClicked: modeCardButton.clicked()
        }
    }

    component SystemOverviewPanel: Item {
        implicitHeight: overviewSect.implicitHeight

        Sect {
            id: overviewSect
            width: parent.width
            title: "SYSTEM OVERVIEW"

            ColumnLayout {
                width: parent.width
                spacing: 8

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: 88
                    radius: 10
                    color: cPanel2
                    border.color: cBorder
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 3

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 6
                            DashboardText {
                                text: "CURRENT INK:"
                                color: cIdle
                                font.pixelSize: 14
                                font.bold: true
                                font.letterSpacing: 1.0
                            }
                            DashboardText {
                                text: tab.inkNameText()
                                color: cText
                                font.pixelSize: 20
                                font.bold: true
                                elide: Text.ElideRight
                                Layout.fillWidth: true
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 6
                            DashboardText {
                                text: "Scan Code:"
                                color: cIdle
                                font.pixelSize: 14
                                font.bold: true
                            }
                            DashboardText {
                                text: tab.inkCodeLotText()
                                color: cMuted
                                font.pixelSize: 15
                                elide: Text.ElideRight
                                Layout.fillWidth: true
                            }
                        }
                    }
                }

                GridLayout {
                    Layout.fillWidth: true
                    columns: 2
                    columnSpacing: 22
                    rowSpacing: 0

                    OverviewKv { lbl: "Mode"; val: tab.modeStr }
                    OverviewKv { lbl: "State"; val: tab.stateStr }
                    OverviewKv { lbl: "Volume"; val: tab.volumeStr }
                    OverviewKv {
                        lbl: "Running"
                        chip: StatusChip {
                            state: tab.running ? "on" : "off"
                            label: tab.running ? "In Process" : "Stop"
                        }
                    }
                    OverviewKv { lbl: "Cycle"; val: tab.cycleStr }
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 5
                    DashboardText {
                        text: "Last Response"
                        color: cMuted
                        font.pixelSize: 18
                        font.bold: true
                    }
                    Rectangle {
                        Layout.fillWidth: true
                        implicitHeight: 44
                        radius: 8
                        color: "#081627"
                        border.color: cBorder
                        border.width: 1
                        DashboardText {
                            anchors.fill: parent
                            anchors.margins: 10
                            text: hpController.manualResponse || "-"
                            color: cText
                            font.pixelSize: 15
                            font.family: monoFamily
                            elide: Text.ElideRight
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }
            }
        }
    }

    component SafetyProcessPanel: Item {
        implicitHeight: safetySect.implicitHeight

        Sect {
            id: safetySect
            width: parent.width
            height: parent.height > 0 ? parent.height : implicitHeight
            title: "SAFETY"

            ColumnLayout {
                width: parent.width
                spacing: 8

                GridLayout {
                    Layout.fillWidth: true
                    columns: 3
                    columnSpacing: 6
                    rowSpacing: 6
                    SafetyBox { lbl: "SAFETY I_4"; ok: tab.safetyOk("safety_i_4_i04") }
                    SafetyBox { lbl: "SAFETY I_5"; ok: tab.safetyOk("safety_i_5_i04") }
                    SafetyBox { lbl: "SAFETY AREA CLEAR"; ok: tab.safetyOk("safety_area_clear") }
                }

                DashboardText {
                    text: "PROCESS"
                    color: cMuted
                    font.pixelSize: 15
                    font.bold: true
                    font.letterSpacing: 1.0
                }

                GridLayout {
                    id: processGrid
                    property real cellWidth: width
                    Layout.fillWidth: true
                    columns: 1
                    columnSpacing: 0
                    rowSpacing: 5
                    ProcessBox {
                        Layout.preferredWidth: processGrid.cellWidth
                        lbl: "AUTO FILL"; val: tab.processAutoText(); active: tab.running && tab.modeStr === "AUTO"
                    }
                    ProcessBox {
                        Layout.preferredWidth: processGrid.cellWidth
                        lbl: "DOSING"; val: tab.processDosingText(); active: tab.running && (tab.modeStr === "AUTO" || tab.modeStr === "CLEAN" || tab.modeStr === "PREFILL")
                    }
                    ProcessBox {
                        Layout.preferredWidth: processGrid.cellWidth
                        lbl: "CLEAN / PREFILL"; val: tab.processCleanText(); active: tab.running && (tab.modeStr === "CLEAN" || tab.modeStr === "PREFILL")
                    }
                    ProcessBox {
                        Layout.preferredWidth: processGrid.cellWidth
                        lbl: "CYCLE / VOLUME"; val: tab.cycleStr + " | " + tab.volumeStr; active: tab.running
                    }
                }
            }
        }
    }

    component SensorGroupCard: Rectangle {
        id: sensorGroupCard
        property string title: ""
        property var items: []
        property int columns: 4
        property int tileHeight: 36
        property bool squareTiles: false
        readonly property real tileSide: squareTiles
                                         ? Math.max(46, Math.floor((Math.max(0, groupCol.width - ((columns - 1) * 4)) / Math.max(1, columns))))
                                         : tileHeight

        Layout.preferredHeight: implicitHeight
        implicitHeight: groupCol.implicitHeight + 16
        radius: 6
        color: Qt.rgba(0.02, 0.08, 0.14, 0.36)
        border.color: cBorder
        border.width: 1

        ColumnLayout {
            id: groupCol
            x: 8; y: 8
            width: parent.width - 16
            spacing: 6

            DashboardText {
                Layout.fillWidth: true
                text: title
                color: cMuted
                font.pixelSize: 14
                font.bold: true
                font.letterSpacing: 0.8
                horizontalAlignment: Text.AlignHCenter
            }

            GridLayout {
                Layout.fillWidth: true
                columns: parent.parent.columns
                columnSpacing: 4
                rowSpacing: 4

                Repeater {
                    model: items
                    SensorTile {
                        sensorKey: modelData
                        Layout.fillWidth: true
                        Layout.preferredHeight: sensorGroupCard.tileSide
                    }
                }
            }
        }
    }

    component SensorTile: Rectangle {
        property string sensorKey: ""
        property bool on_: classifyState(tab.inputsMap[sensorKey]) === "on"

        radius: 4
        color: "transparent"
        border.color: on_ ? tab.cSensorActiveBorder : tab.cSensorIdleBorder
        border.width: on_ ? 2 : 1
        opacity: on_ ? 1.0 : 0.78
        gradient: on_ ? sensorActiveGradient : sensorIdleGradient
        Behavior on border.color { ColorAnimation { duration: 150 } }

        Gradient {
            id: sensorActiveGradient
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: tab.cSensorActiveStart }
            GradientStop { position: 1.0; color: tab.cSensorActiveEnd }
        }
        Gradient {
            id: sensorIdleGradient
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: tab.cSensorIdleBg }
            GradientStop { position: 1.0; color: Qt.rgba(0.02, 0.07, 0.12, 0.10) }
        }

        Column {
            anchors.centerIn: parent
            width: parent.width - 6
            spacing: 2
            DashboardText {
                width: parent.width
                text: getSensorLabel(sensorKey)
                color: parent.parent.on_ ? tab.cSensorActiveText : tab.cSensorIdleText
                font.pixelSize: Math.min(14, Math.max(10, Math.min(parent.parent.width, parent.parent.height) * 0.15))
                font.bold: true
                wrapMode: Text.WrapAnywhere
                horizontalAlignment: Text.AlignHCenter
                maximumLineCount: 2
                elide: Text.ElideRight
            }
            Rectangle {
                width: 5; height: 5; radius: 3
                color: parent.parent.on_ ? tab.cSensorActiveText : tab.cSensorIdleDot
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }
    }

    component OperationalSettingsPanel: Item {
        implicitHeight: settingsSect.implicitHeight

        Sect {
            id: settingsSect
            width: parent.width
            height: parent.height > 0 ? parent.height : implicitHeight
            title: "OPERATING PARAMETERS"
            fillBodyHeight: true
            frameBorderWidth: 0

            ColumnLayout {
                anchors.fill: parent
                spacing: 7

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 34
                    DashboardText {
                        text: "BASE PWM SUGGESTION"
                        color: cAccent
                        font.pixelSize: 15
                        font.bold: true
                        font.letterSpacing: 1.0
                    }
                    Item { Layout.fillWidth: true }
                    CompactActionBtn {
                        lbl: "Restore Defaults"
                        variant: "warn"
                        Layout.preferredWidth: 180
                        labelPixelSize: 13
                        Layout.preferredHeight: 34
                        onClicked: hpController.publishString("parameters_control", "reset_defaults")
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: 58
                    radius: 8
                    color: cField
                    border.color: cBorder
                    border.width: 1

                    Rectangle {
                        width: 3
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        anchors.topMargin: 10
                        anchors.bottomMargin: 10
                        anchors.leftMargin: 8
                        radius: 2
                        color: cWarn
                    }

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 18
                        anchors.rightMargin: 10
                        spacing: 6

                        DashboardText {
                            text: "51.5ml @ 2.0ml/s -> recommended Base PWM 37% (current " + hpController.basePwmStatus + "%). Apply 37%?"
                            color: cText
                            opacity: 0.88
                            font.pixelSize: 15
                            elide: Text.ElideRight
                            Layout.fillWidth: true
                        }
                        CompactActionBtn {
                            lbl: "Apply 37%"
                            variant: "action"
                            labelPixelSize: 14
                            Layout.preferredWidth: 126
                            Layout.preferredHeight: 38
                            onClicked: hpController.publishInt("base_pwm", 37)
                        }
                        CompactActionBtn {
                            lbl: "Skip"
                            labelPixelSize: 14
                            Layout.preferredWidth: 90
                            Layout.preferredHeight: 38
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    spacing: 5
                    Repeater {
                        model: tab.settingGroups
                        CompactActionBtn {
                            lbl: modelData.label
                            variant: tab.activeSettingsTab === index ? "action" : "default"
                            labelPixelSize: 15
                            Layout.preferredWidth: Math.max(96, implicitWidth + 10)
                            Layout.preferredHeight: 40
                            onClicked: tab.activeSettingsTab = index
                        }
                    }
                    Item { Layout.fillWidth: true }
                }

                GridLayout {
                    id: settingsGrid
                    property real cellWidth: Math.max(0, (width - columnSpacing) / columns)
                    property int itemCount: tab.settingGroups[tab.activeSettingsTab].items.length
                    property int rowCount: Math.max(1, Math.ceil(itemCount / columns))
                    property real rowHeight: rowCount > 0
                                             ? Math.max(42, Math.floor((height - ((rowCount - 1) * rowSpacing)) / rowCount))
                                             : 42
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    columns: 2
                    columnSpacing: 7
                    rowSpacing: 4

                    Repeater {
                        model: tab.settingGroups[tab.activeSettingsTab].items
                        CompactSettingRow {
                            Layout.fillWidth: true
                            Layout.preferredWidth: settingsGrid.cellWidth
                            Layout.preferredHeight: settingsGrid.rowHeight
                            item: modelData
                            currentVal: tab.settingCurrentValue(modelData)
                        }
                    }
                }

            }
        }
    }

    component CompactSettingRow: Rectangle {
        property var item: ({})
        property string currentVal: ""
        readonly property string currentDisplay: (currentVal || "-") + (item.unit ? " " + item.unit : "")

        radius: 6
        color: cField
        border.color: cBorder
        border.width: 1

        RowLayout {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            anchors.leftMargin: 8
            anchors.rightMargin: 8
            height: Math.min(38, Math.max(30, parent.height - 10))
            spacing: 6

            DashboardText {
                text: tab.settingDisplayLabel(item)
                color: cText
                font.pixelSize: 14
                font.bold: true
                font.letterSpacing: 0
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
                Layout.preferredWidth: Math.max(112, Math.min(148, parent.parent.width * 0.27))
                Layout.minimumWidth: 104
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            Rectangle {
                id: compactInputFrame
                Layout.preferredWidth: Math.max(82, Math.min(96, parent.parent.width * 0.16))
                Layout.preferredHeight: parent.height
                Layout.alignment: Qt.AlignVCenter
                radius: 5
                color: "#07131f"
                border.color: compactInp.activeFocus ? cBtnActionStart : cBorder
                border.width: compactInp.activeFocus ? 2 : 1
                Behavior on border.color { ColorAnimation { duration: 120 } }
                SmartTextInput {
                    id: compactInp
                    focusHost: tab.focusHost
                    showFocusBorder: false
                    anchors.fill: parent
                    anchors.leftMargin: 10
                    anchors.rightMargin: 10
                    text: currentVal
                    color: cText
                    font.pixelSize: 17
                    font.bold: true
                    font.family: monoFamily
                    selectByMouse: true
                    horizontalAlignment: TextInput.AlignHCenter
                    verticalAlignment: TextInput.AlignVCenter
                }
            }

            Rectangle {
                Layout.minimumWidth: 138
                Layout.preferredWidth: Math.max(158, Math.min(184, parent.parent.width * 0.30))
                Layout.preferredHeight: parent.height
                Layout.alignment: Qt.AlignVCenter
                radius: 5
                color: "#0a1a2b"
                border.color: cBorder
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 10
                    anchors.rightMargin: 8
                    spacing: 3

                    DashboardText {
                        text: "USED"
                        color: cIdle
                        font.pixelSize: 11
                        font.bold: true
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                        Layout.preferredWidth: 36
                        Layout.alignment: Qt.AlignVCenter
                    }
                    DashboardText {
                        text: currentDisplay
                        color: cAccent
                        font.pixelSize: 16
                        font.bold: true
                        font.family: monoFamily
                        elide: Text.ElideNone
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignVCenter
                    }
                }
            }

            CompactActionBtn {
                lbl: "SET"
                variant: "action"
                labelPixelSize: 12
                Layout.preferredWidth: 84
                Layout.preferredHeight: Math.min(32, parent.height)
                Layout.alignment: Qt.AlignVCenter
                onClicked: tab.publishSetting(item, compactInp.text)
            }
        }
    }

    component CompactActionBtn: Rectangle {
        id: compactBtn
        property string lbl: "Btn"
        property string variant: "default"
        property int labelPixelSize: 11
        signal clicked

        readonly property color gradStart: variant === "primary" ? cBtnPrimaryStart
                                          : variant === "warn" ? cBtnWarnStart
                                          : variant === "action" ? (compactMouse.pressed ? cBtnActionPressStart : (compactMouse.containsMouse ? cBtnActionHoverStart : cBtnActionStart))
                                          : cBtnBaseStart
        readonly property color gradEnd: variant === "primary" ? cBtnPrimaryEnd
                                        : variant === "warn" ? cBtnWarnEnd
                                        : variant === "action" ? (compactMouse.pressed ? cBtnActionPressEnd : (compactMouse.containsMouse ? cBtnActionHoverEnd : cBtnActionEnd))
                                        : cBtnBaseEnd
        implicitWidth: Math.max(62, labelText.implicitWidth + 24)
        implicitHeight: 34
        radius: height / 2
        color: gradStart
        border.color: variant === "primary" ? cOk : (variant === "warn" ? cWarn : (variant === "action" ? cBtnActionEnd : cBorder))
        border.width: 1

        Rectangle {
            anchors.fill: parent
            radius: parent.radius
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.0; color: compactBtn.gradStart }
                GradientStop { position: 1.0; color: compactBtn.gradEnd }
            }
        }
        DashboardText {
            id: labelText
            anchors.centerIn: parent
            text: lbl
            color: "#ffffff"
            font.pixelSize: labelPixelSize
            font.bold: true
            elide: Text.ElideRight
            width: parent.width - 14
            horizontalAlignment: Text.AlignHCenter
        }
        MotionMouseArea {
            id: compactMouse
            anchors.fill: parent
            cursorShape: Qt.PointingHandCursor
            hoverScale: 1.012
            pressScale: 0.99
            shadowEnabled: false
            shimmerEnabled: false
            raiseOnHover: true
            onClicked: compactBtn.clicked()
        }
    }

    component OverviewKv: Item {
        property string lbl: ""
        property string val: ""
        property Item chip: null
        Layout.fillWidth: true
        Layout.preferredHeight: 40

        Rectangle {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            height: 1
            color: cBorder
        }
        DashboardText {
            anchors.left: parent.left
            anchors.verticalCenter: parent.verticalCenter
            text: lbl
            color: cMuted
            font.pixelSize: 17
        }
        Loader {
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            sourceComponent: chip ? chipWrap : textComp
            Component {
                id: textComp
                DashboardText {
                    text: val
                    color: cText
                    font.pixelSize: 18
                    font.bold: true
                    horizontalAlignment: Text.AlignRight
                }
            }
            Component {
                id: chipWrap
                Item { width: childrenRect.width; height: childrenRect.height; children: chip ? [chip] : [] }
            }
        }
    }

    component SafetyBox: Rectangle {
        property string lbl: ""
        property bool ok: false
        Layout.fillWidth: true
        Layout.preferredHeight: 58
        radius: 10
        color: ok ? cOkBg : cBadBg
        border.width: 0
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 6
            spacing: 1
            DashboardText {
                Layout.fillWidth: true
                Layout.fillHeight: true
                text: lbl
                color: cIdle
                font.pixelSize: 10
                font.bold: true
                wrapMode: Text.WordWrap
                maximumLineCount: 2
                elide: Text.ElideNone
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            DashboardText {
                Layout.fillWidth: true
                text: ok ? "OK" : "BLOCK"
                color: ok ? cOk : cBad
                font.pixelSize: 15
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
            }
        }
    }

    component ProcessBox: Rectangle {
        id: processBox
        property string lbl: ""
        property string val: "-"
        property bool active: false
        Layout.fillWidth: true
        Layout.preferredHeight: 48
        radius: 10
        color: "transparent"
        border.color: cBorder
        border.width: active ? 0 : 1
        gradient: active ? processActiveGradient : processIdleGradient
        Gradient {
            id: processActiveGradient
            orientation: Gradient.Horizontal
            // Same selected palette as Robot Control's CONTINUOUS JOG.
            GradientStop { position: 0.0; color: "#163a52" }
            GradientStop { position: 1.0; color: "#081627" }
        }
        Gradient {
            id: processIdleGradient
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: cPanel2 }
            GradientStop { position: 1.0; color: cPanel2 }
        }
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 14
            anchors.rightMargin: 14
            spacing: 8
            DashboardText {
                text: lbl
                color: cIdle
                font.pixelSize: 15
                font.bold: true
                elide: Text.ElideRight
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
                Layout.preferredWidth: 126
                Layout.fillHeight: true
            }
            DashboardText {
                text: "-"
                color: cMuted
                font.pixelSize: 16
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                Layout.preferredWidth: 14
                Layout.fillHeight: true
            }
            DashboardText {
                text: val
                color: cText
                font.pixelSize: 18
                font.bold: true
                font.family: monoFamily
                elide: Text.ElideRight
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
        }
    }

    component ManualControlsPage: Item {
        implicitHeight: Math.max(820, bodyScrollView.height - 24)

        RowLayout {
            anchors.fill: parent
            spacing: 12

            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 12

                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    spacing: 12

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 12

                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            Layout.preferredWidth: 860
                            spacing: 12

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                spacing: 12

                                Sect {
                                    title: "Cylinders (manual only)"
                                    fillBodyHeight: true
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 500
                                    Layout.fillHeight: true
                                    Layout.alignment: Qt.AlignTop

                                    ColumnLayout {
                                        anchors.fill: parent
                                        enabled: tab.modeStr === "MANUAL"
                                        opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                        spacing: 8
                                        Behavior on opacity { NumberAnimation { duration: 150 } }
                                        Repeater {
                                            model: tab.cylinderModel
                                            IoToggle {
                                                Layout.fillWidth: true
                                                Layout.fillHeight: true
                                                Layout.minimumHeight: 56
                                                ioId:      modelData.id
                                                statusKey: modelData.statusKey
                                                ioLabel:   modelData.label
                                                actA:      modelData.a
                                                actB:      modelData.b
                                            }
                                        }
                                    }
                                }

                                Sect {
                                    title: "Servo & Motor"
                                    fillBodyHeight: true
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 360
                                    Layout.fillHeight: true
                                    Layout.alignment: Qt.AlignTop

                                    ColumnLayout {
                                        anchors.fill: parent
                                        spacing: 10

                                        Rectangle {
                                            Layout.fillWidth: true
                                            Layout.preferredHeight: 58
                                            radius: 8
                                            color: cField
                                            border.color: cBorder
                                            border.width: 1
                                            DashboardText {
                                                anchors.centerIn: parent
                                                text: "Pos: " + hpController.servoPosition.toFixed(2) + " mm"
                                                color: cText
                                                font.pixelSize: 21
                                                font.bold: true
                                                font.family: monoFamily
                                            }
                                        }

                                        ColumnLayout {
                                            spacing: 8
                                            Layout.fillWidth: true

                                            TbBtn { lbl: "Enable"; Layout.fillWidth: true; Layout.preferredHeight: 46; onClicked: hpController.publishString("servo_command", "enable") }
                                            TbBtn { lbl: "Disable"; Layout.fillWidth: true; Layout.preferredHeight: 46; onClicked: hpController.publishString("servo_command", "disable") }
                                            TbBtn { lbl: "Home"; variant: "primary"; Layout.fillWidth: true; Layout.preferredHeight: 46; onClicked: hpController.publishString("servo_command", "home") }
                                            TbBtn { lbl: "Reset Fault"; variant: "danger"; Layout.fillWidth: true; Layout.preferredHeight: 46; onClicked: hpController.publishString("servo_command", "reset_fault") }
                                        }

                                        ColumnLayout {
                                            spacing: 8
                                            Layout.fillWidth: true
                                            enabled: tab.modeStr === "MANUAL"
                                            opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                            Behavior on opacity { NumberAnimation { duration: 150 } }

                                            RowLayout {
                                                spacing: 8
                                                Layout.fillWidth: true
                                                JogBtn { lbl: "JOG REV"; dir: "rev"; variant: "warn"; Layout.fillWidth: true; Layout.preferredHeight: 54 }
                                                JogBtn { lbl: "JOG FWD"; dir: "fwd"; variant: "primary"; Layout.fillWidth: true; Layout.preferredHeight: 54 }
                                            }
                                            TbBtn {
                                                lbl: "STOP"
                                                variant: "danger"
                                                Layout.fillWidth: true
                                                Layout.preferredHeight: 48
                                                onClicked: hpController.publishString("servo_jog", "stop")
                                            }
                                        }

                                        Item { Layout.fillHeight: true }

                                        ColumnLayout {
                                            Layout.fillWidth: true
                                            spacing: 8

                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 8
                                                DashboardText { text: "Base PWM"; color: cText; font.pixelSize: 17; font.bold: true; Layout.fillWidth: true }
                                                PwmInput { id: basePwmIn; valueText: hpController.basePwmStatus.toString(); Layout.preferredWidth: 70 }
                                                TbBtn {
                                                    lbl: "Set"; variant: "primary"
                                                    Layout.preferredWidth: 70
                                                    Layout.preferredHeight: 38
                                                    onClicked: {
                                                        var v = parseInt(basePwmIn.valueText);
                                                        if (!isNaN(v)) hpController.publishInt("base_pwm", Math.max(0, Math.min(100, v)));
                                                    }
                                                }
                                            }
                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 8
                                                enabled: tab.modeStr === "MANUAL"
                                                opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                                DashboardText { text: "V10 PWM"; color: cText; font.pixelSize: 17; font.bold: true; Layout.fillWidth: true }
                                                PwmInput { id: v10In; valueText: (tab.valvesMap["v10"] && tab.valvesMap["v10"].label) ? tab.valvesMap["v10"].label.replace("%","") : "0"; Layout.preferredWidth: 70 }
                                                TbBtn {
                                                    lbl: "Set"; variant: "primary"
                                                    Layout.preferredWidth: 70
                                                    Layout.preferredHeight: 38
                                                    onClicked: {
                                                        var v = parseInt(v10In.valueText);
                                                        if (!isNaN(v)) hpController.publishManual("valve10", String(Math.max(0, Math.min(100, v))));
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 210
                                spacing: 12

                                Sect {
                                    title: "Action log"
                                    fillBodyHeight: true
                                    Layout.preferredWidth: 500
                                    Layout.maximumWidth: 500
                                    Layout.fillHeight: true
                                    ColumnLayout {
                                        anchors.fill: parent
                                        spacing: 4
                                        RowLayout {
                                            Layout.fillWidth: true
                                            DashboardText { text: tab.actionLog.length > 0 ? (tab.actionLog.length + " recent actions") : "No actions yet"; color: cMuted; font.pixelSize: 17 }
                                            Item { Layout.fillWidth: true }
                                            TbBtn { lbl: "Clear log"; visible: tab.actionLog.length > 0; onClicked: { tab.actionLog = []; tab.lastActionRaw = "" } }
                                        }
                                        ScrollView {
                                            Layout.fillWidth: true
                                            Layout.fillHeight: true
                                            clip: true
                                            ColumnLayout {
                                                width: parent.width
                                                spacing: 2
                                                Repeater {
                                                    model: tab.actionLog
                                                    DashboardText {
                                                        text: modelData; color: cText
                                                        font.pixelSize: 16; font.family: monoFamily
                                                        width: parent.width; elide: Text.ElideRight
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                RawStatusPanel {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 360
                                    Layout.fillHeight: true
                                }
                            }
                        }

                        Sect {
                            title: "Valves (manual only)"
                            fillBodyHeight: true
                            Layout.fillWidth: true
                            Layout.preferredWidth: 450
                            Layout.fillHeight: true
                            Layout.alignment: Qt.AlignTop
                            ColumnLayout {
                                anchors.fill: parent
                                enabled: tab.modeStr === "MANUAL"
                                opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                spacing: 8
                                Behavior on opacity { NumberAnimation { duration: 150 } }
                                Repeater {
                                    model: tab.valveModel
                                    IoToggle {
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true
                                        Layout.minimumHeight: 48
                                        ioId:      modelData.id
                                        statusKey: modelData.statusKey
                                        ioLabel:   modelData.label
                                        actA:      modelData.a
                                        actB:      modelData.b
                                    }
                                }
                            }
                        }
                    }
                }
            }

            PressureMonitorPanel {
                Layout.preferredWidth: 400
                Layout.maximumWidth: 400
                Layout.fillHeight: true
                Layout.alignment: Qt.AlignRight
            }
        }
    }

    component RawStatusPanel: Rectangle {
        color: cPanel
        radius: 6
        border.color: cBorder
        border.width: 1
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? cControlHover : cBorder }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 8

            DashboardText {
                text: "Raw Status"
                color: cMuted
                font.pixelSize: 18
                font.bold: true
                font.letterSpacing: 0.6
                Layout.fillWidth: true
            }

            ScrollView {
                id: rawStatusScroll
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                contentWidth: availableWidth
                ScrollBar.vertical.policy: ScrollBar.AlwaysOn
                TextEdit {
                    width: rawStatusScroll.availableWidth - 8
                    readOnly: true
                    selectByMouse: true
                    textFormat: TextEdit.PlainText
                    text: "system_status=" + (hpController.systemStatus || "-")
                          + "\nmode_status=" + (hpController.modeStatus || "-")
                          + "\nink_status=" + (hpController.inkStatus || "-")
                          + "\ndosing_status=" + (hpController.dosingStatus || "-")
                          + "\nfill_status=" + (hpController.fillStatus || "-")
                          + "\nfix_status=" + (hpController.fixStatus || "-")
                          + "\ncr_status=" + (hpController.crStatus || "-")
                          + "\nhw_status=" + (hpController.hwStatus || "-")
                          + "\ninput_state=" + (hpController.inputState || "-")
                          + "\nvalve_state=" + (hpController.valveState || "-")
                          + "\nthresholds=" + (hpController.pressureThresholds || "-")
                          + "\nmanual_response=" + (hpController.manualResponse || "-")
                          + "\nbase_pwm=" + hpController.basePwmStatus
                          + "\nservo_position=" + hpController.servoPosition.toFixed(2)
                          + "\npressure_s1=" + hpController.pressureS1.toFixed(1)
                          + "\npressure_s2=" + hpController.pressureS2.toFixed(1)
                          + "\npressure_s3=" + hpController.pressureS3.toFixed(1)
                          + "\ncartridge_pressures=" + JSON.stringify(hpController.cartridgePressures)
                    color: cText
                    font.pixelSize: 12
                    font.family: monoFamily
                    wrapMode: TextEdit.WrapAnywhere
                }
            }
        }
    }

    component PressureMonitorPanel: Rectangle {
        radius: 6
        color: cPanel
        border.color: cBorder
        border.width: 1
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? cControlHover : cBorder }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            spacing: 10

            DashboardText {
                text: "ANALOG PRESSURE"
                color: cText
                font.pixelSize: 22
                font.bold: true
                font.letterSpacing: 0.6
                Layout.fillWidth: true
            }

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 8
                PCard { lbl: "S1 Chamber";   val: hpController.pressureS1; maxVal: 1200 }
                PCard { lbl: "S2 Cartridge"; val: hpController.pressureS2; maxVal: 1200 }
                PCard { lbl: "S3 Tank";      val: hpController.pressureS3; maxVal: 1200 }
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.topMargin: 6
                DashboardText {
                    text: "CARTRIDGE\nPRESSURE"
                    color: cMuted
                    font.pixelSize: 18
                    font.bold: true
                    Layout.fillWidth: true
                }
                DashboardText {
                    text: tab.getCartridgeStats()
                    color: cMuted
                    font.pixelSize: 15
                    font.bold: true
                    horizontalAlignment: Text.AlignRight
                    wrapMode: Text.Wrap
                    Layout.preferredWidth: 220
                }
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 5
                Repeater {
                    model: 8
                    CartRow {
                        Layout.fillHeight: true
                        cartName: "Cart " + (index + 1)
                        cartVal: (hpController.cartridgePressures && hpController.cartridgePressures.length > index) ? (Number(hpController.cartridgePressures[index]) || 0) : 0
                    }
                }
            }
        }
    }

    component Sect: Rectangle {
        property alias title: ttl.text
        property color bgColor: cPanel
        property color borderColor: cBorder
        property bool  noTitle: false
        property bool  fillBodyHeight: false
        property real  frameBorderWidth: 1
        default property alias contentChildren: ci.children

        color: bgColor; radius: 6
        border.color: borderColor; border.width: frameBorderWidth
        implicitHeight: inner.implicitHeight + 24
        Layout.preferredHeight: implicitHeight
        HoverHandler { enabled: parent.frameBorderWidth > 0; onHoveredChanged: parent.border.color = hovered ? cControlHover : borderColor }

        ColumnLayout {
            id: inner
            x: 12; y: 12
            width: parent.width - 24
            height: parent.height - 24
            spacing: 10
            DashboardText {
                id: ttl
                visible: !noTitle && text.length > 0
                color: cMuted; font.pixelSize: 22; font.bold: true
                font.letterSpacing: 0.6
                text: ""
            }
            Item {
                id: ci
                Layout.fillWidth: true
                Layout.fillHeight: fillBodyHeight
                implicitHeight: childrenRect.height
            }
        }
    }

    component TbBtn: Rectangle {
        id: tbBtn
        property string lbl: "Btn"
        property string variant: "default"
        property url iconSource: ""
        signal clicked

        readonly property color baseBorder: variant === "primary" ? cOk
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad
                                          : variant === "action"  ? cAccent : "#163a52"
        readonly property color gradStart:  variant === "primary" ? cBtnPrimaryStart
                                          : variant === "warn"    ? cBtnWarnStart
                                          : variant === "danger"  ? cBtnDangerStart
                                          : variant === "action"  ? cBtnActionStart : cBtnBaseStart
        readonly property color gradEnd:    variant === "primary" ? cBtnPrimaryEnd
                                          : variant === "warn"    ? cBtnWarnEnd
                                          : variant === "danger"  ? cBtnDangerEnd
                                          : variant === "action"  ? cBtnActionEnd : cBtnBaseEnd
        readonly property color baseFg:     "#ffffff"
        readonly property color currentStart: ma.pressed ? Qt.darker(gradStart, 1.18) : (ma.containsMouse ? Qt.lighter(gradStart, 1.08) : gradStart)
        readonly property color currentEnd:   ma.pressed ? Qt.darker(gradEnd, 1.18) : (ma.containsMouse ? Qt.lighter(gradEnd, 1.08) : gradEnd)

        implicitWidth: Math.max(80, buttonContent.implicitWidth + 26)
        implicitHeight: 38
        radius: 8
        color: currentStart
        border.color: "transparent"
        border.width: 0
        Rectangle {
            anchors.fill: parent
            radius: parent.radius
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.0; color: tbBtn.currentStart }
                GradientStop { position: 1.0; color: tbBtn.currentEnd }
            }
        }

        Row {
            id: buttonContent
            anchors.centerIn: parent
            spacing: tbBtn.iconSource.toString().length > 0 ? 8 : 0
            Image {
                visible: tbBtn.iconSource.toString().length > 0
                source: tbBtn.iconSource
                width: visible ? 20 : 0
                height: visible ? 20 : 0
                sourceSize.width: 40
                sourceSize.height: 40
                fillMode: Image.PreserveAspectFit
                smooth: true
                anchors.verticalCenter: parent.verticalCenter
            }
            DashboardText {
                id: t
                text: lbl; color: baseFg
                font.pixelSize: 21; font.bold: true
                anchors.verticalCenter: parent.verticalCenter
            }
        }
        MotionMouseArea {
            id: ma; anchors.fill: parent
            hoverEnabled: true; cursorShape: Qt.PointingHandCursor
            hoverScale: 1.012
            pressScale: 0.99
            shadowEnabled: false
            shimmerEnabled: false
            raiseOnHover: true
            onClicked: parent.clicked()
        }
    }

    component ModeBtn: TbBtn {
        property bool active: false
        variant: active ? "action" : "default"
    }

    component CycleChip: Rectangle {
        property string value: "00:00"
        property bool   isRun: false
        implicitWidth: Math.max(120, valText.implicitWidth + 24)
        implicitHeight: 38
        radius: 6
        color: isRun ? cOkBg : cIdleBg
        border.color: isRun ? cOk : cBorder; border.width: 1
        DashboardText {
            id: valText
            anchors.centerIn: parent; text: value
            color: parent.isRun ? cOk : cText
            font.pixelSize: 21; font.bold: true; font.family: monoFamily
        }
    }

    component StatusChip: Rectangle {
        property string state: "mid"
        property string label: state
        implicitWidth: txt.implicitWidth + 24; implicitHeight: 34
        radius: 17
        color:        state === "on" ? cOkBg : state === "off" ? cBadBg : state === "mid" ? cWarnBg : cIdleBg
        border.width: 0
        DashboardText {
            id: txt; anchors.centerIn: parent; text: label
            color: parent.state === "on" ? cOk : parent.state === "off" ? cBad : parent.state === "mid" ? cWarn : cIdle
            font.pixelSize: 19; font.bold: true
        }
    }

    component Kv: RowLayout {
        property string lbl: ""
        property string val: ""
        property Item   chip: null
        width: parent.width; spacing: 8
        DashboardText {
            text: lbl; color: cMuted
            font.pixelSize: 20; font.bold: true
            Layout.preferredWidth: 170
        }
        Loader {
            Layout.fillWidth: true
            sourceComponent: chip ? chipWrap : textComp
            Component {
                id: textComp
                DashboardText {
                    text: val; color: cText
                    font.pixelSize: 21
                    wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                }
            }
            Component {
                id: chipWrap
                Item { width: childrenRect.width; height: childrenRect.height; children: chip ? [chip] : [] }
            }
        }
    }

    component Chip: Rectangle {
        property string name: ""
        property string state: "mid"
        property string label: state
        implicitWidth: Math.min(260, Math.max(170, nameTxt.implicitWidth + sChip.implicitWidth + 32))
        implicitHeight: 40
        radius: 6
        color: cPanel2; border.color: cBorder; border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 8; anchors.rightMargin: 8
            spacing: 6
            DashboardText {
                id: nameTxt
                text: name; color: cText
                font.pixelSize: 20
                elide: Text.ElideRight; Layout.fillWidth: true
            }
            StatusChip { id: sChip; state: parent.parent.state; label: parent.parent.label }
        }
    }

    component PCard: Rectangle {
        property string lbl: ""
        property real   val: 0
        property real   maxVal: 1000
        readonly property string cls: classifyPressure(val, maxVal * 0.18, maxVal * 0.35, maxVal * 0.90)
        Layout.fillWidth: true
        implicitHeight: pc.implicitHeight + 24
        radius: 8
        color: cPanel2
        border.color: "#163a52"
        border.width: 1
        clip: true
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? cAccent : "#163a52" }
        ColumnLayout {
            id: pc; x: 12; y: 12
            width: parent.width - 24; spacing: 8
            RowLayout {
                width: parent.width
                DashboardText { text: lbl; color: cText; font.pixelSize: 18; font.bold: true }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 3
                    DashboardText { text: val.toFixed(1); color: pressureTextColor(cls); font.pixelSize: 28; font.bold: true; font.family: monoFamily }
                    DashboardText { text: "mbar"; color: pressureTextColor(cls); font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignBottom; Layout.bottomMargin: 3 }
                }
            }
            Rectangle {
                Layout.fillWidth: true; height: 12; radius: height / 2
                color: pressureTrackColor(cls)
                border.color: "#22445c"
                border.width: 1
                clip: true
                Rectangle {
                    height: parent.height; radius: parent.radius
                    width: parent.width * Math.max(0, Math.min(1, val / maxVal))
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: pressureFillStart(cls) }
                        GradientStop { position: 0.55; color: "#1e9ed0" }
                        GradientStop { position: 1.0; color: pressureFillEnd(cls) }
                    }
                }
            }
        }
    }

    component CartRow: Rectangle {
        property string cartName: ""
        property real   cartVal: 0
        readonly property string cls: classifyPressure(cartVal, 280, 400, 600)
        Layout.fillWidth: true
        implicitHeight: 58
        radius: 6
        color: cPanel2
        clip: true
        border.color: "#163a52"
        border.width: 1
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? cAccent : "#163a52" }
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10; anchors.rightMargin: 10
            spacing: 8
            DashboardText {
                text: cartName; color: cText
                font.pixelSize: 20; font.bold: true
                Layout.preferredWidth: 95
            }
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                RowLayout {
                    width: parent.width
                    Item { Layout.fillWidth: true }
                    DashboardText { text: cartVal.toFixed(1); color: pressureTextColor(cls); font.pixelSize: 22; font.bold: true; font.family: monoFamily }
                    DashboardText { text: "mbar"; color: pressureTextColor(cls); font.pixelSize: 13; font.bold: true; Layout.alignment: Qt.AlignBottom; Layout.bottomMargin: 2 }
                }
                Rectangle {
                    Layout.fillWidth: true; height: 10; radius: height / 2
                    color: pressureTrackColor(cls)
                    border.color: "#22445c"
                    border.width: 1
                    clip: true
                    Rectangle {
                        height: parent.height; radius: parent.radius
                        width: parent.width * Math.max(0, Math.min(1, cartVal / 1200))
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: pressureFillStart(cls) }
                            GradientStop { position: 0.55; color: "#1e9ed0" }
                            GradientStop { position: 1.0; color: pressureFillEnd(cls) }
                        }
                    }
                }
            }
        }
    }

    component PCardBox: Rectangle {
        property string lbl: ""
        property string val: ""
        property bool   active: false
        Layout.preferredHeight: 92
        radius: 8
        color: active ? cAccentSoft : cPanel2
        border.color: active ? cAccent : cBorder
        border.width: active ? 3 : 1
        Behavior on border.width { NumberAnimation { duration: 180 } }
        // Glow effect when active
        Rectangle {
            visible: parent.active
            anchors.fill: parent
            radius: parent.radius
            color: "transparent"
            border.color: cAccent
            border.width: 1
            opacity: 0.5
            anchors.margins: -2
        }
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 5
            DashboardText { text: lbl; color: parent.parent.active ? cAccent : cMuted; font.pixelSize: 20; font.bold: parent.parent.active }
            DashboardText { text: val; color: cText; font.pixelSize: 23; font.bold: true; wrapMode: Text.Wrap }
        }
    }

    // Alert row component for Trung tam canh bao
    component AlertRow: Rectangle {
        property string sev: "info"
        property string time: ""
        property string area: ""
        property string message: ""
        Layout.fillWidth: true
        implicitHeight: arCol.implicitHeight + 18
        radius: 6
        color:        sev === "critical" ? Qt.rgba(1.0, 0.15, 0.15, 0.18)
                    : sev === "error"    ? Qt.rgba(1.0, 0.32, 0.32, 0.10)
                    : sev === "warning"  ? Qt.rgba(1.0, 0.65, 0.15, 0.10)
                    : Qt.rgba(0.42, 0.75, 0.95, 0.10)
        border.color: sev === "critical" ? cBad
                    : sev === "error"    ? cBad
                    : sev === "warning"  ? cWarn
                    : cAccent
        border.width: 1

        RowLayout {
            id: arCol
            anchors.fill: parent
            anchors.margins: 10
            spacing: 10

            DashboardText {
                text: sev === "critical" ? "🔥" : sev === "error" ? "⛔" : sev === "warning" ? "⚠️" : "ℹ️"
                font.pixelSize: 22
            }
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                RowLayout {
                    width: parent.width
                    DashboardText {
                        text: (area ? area + " · " : "") + sev.toUpperCase()
                        color: parent.parent.parent.parent.parent.sev === "warning" ? cWarn
                             : parent.parent.parent.parent.parent.sev === "info"    ? cAccent : cBad
                        font.pixelSize: 14; font.bold: true
                    }
                    Item { Layout.fillWidth: true }
                    DashboardText {
                        text: time; color: cMuted
                        font.pixelSize: 13; font.family: monoFamily
                    }
                }
                DashboardText {
                    text: message; color: cText
                    font.pixelSize: 15
                    wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                    Layout.fillWidth: true
                }
            }
        }
    }

    // Single valve/cylinder row: label + 2 toggle buttons (a/b) — compact
    component IoToggle: Rectangle {
        id: ioToggle
        property string ioId: ""
        property string statusKey: ""
        property string ioLabel: ""
        property string actA: "on"
        property string actB: "off"
        readonly property var entry: tab.valvesMap[statusKey] || null
        readonly property string curLabel: entry ? entry.label : ""
        readonly property bool aActive: actionMatches(curLabel, actA)
        readonly property bool bActive: actionMatches(curLabel, actB)
        implicitWidth: 360
        implicitHeight: 48
        radius: 6
        color: cField; border.color: cBorder; border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 12; anchors.rightMargin: 8
            spacing: 8
            DashboardText {
                text: ioLabel; color: cText
                font.pixelSize: 16; font.bold: true
                Layout.fillWidth: true; elide: Text.ElideRight
            }
            // Action A button
            Rectangle {
                id: actionABtn
                property bool held: false
                property bool hovered: false
                readonly property color startColor: held ? cBtnActionPressStart : (hovered ? cBtnActionHoverStart : (ioToggle.aActive ? cIoActiveStart : cBtnBaseStart))
                readonly property color endColor: held ? cBtnActionPressEnd : (hovered ? cBtnActionHoverEnd : (ioToggle.aActive ? cIoActiveEnd : cBtnBaseEnd))
                Layout.preferredWidth: 76; Layout.preferredHeight: 34; radius: 8
                color: startColor
                border.color: "transparent"; border.width: 0
                Rectangle {
                    anchors.fill: parent
                    radius: parent.radius
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: actionABtn.startColor }
                        GradientStop { position: 1.0; color: actionABtn.endColor }
                    }
                }
                DashboardText {
                    anchors.centerIn: parent; text: actA
                    color: ioToggle.aActive ? cIoActiveText : "#ffffff"
                    font.pixelSize: 16; font.bold: true
                }
                MotionMouseArea {
                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    hoverScale: 1.012
                    pressScale: 0.99
                    shadowEnabled: false
                    shimmerEnabled: false
                    raiseOnHover: true
                    onPressed: actionABtn.held = true
                    onReleased: actionABtn.held = false
                    onCanceled: actionABtn.held = false
                    onEntered: actionABtn.hovered = true
                    onExited: { actionABtn.hovered = false; actionABtn.held = false }
                    onClicked: hpController.publishManual(ioId, actA)
                }
            }
            Rectangle {
                id: actionBBtn
                property bool held: false
                property bool hovered: false
                readonly property color startColor: held ? cBtnActionPressStart : (hovered ? cBtnActionHoverStart : (ioToggle.bActive ? cIoActiveStart : cBtnBaseStart))
                readonly property color endColor: held ? cBtnActionPressEnd : (hovered ? cBtnActionHoverEnd : (ioToggle.bActive ? cIoActiveEnd : cBtnBaseEnd))
                Layout.preferredWidth: 76; Layout.preferredHeight: 34; radius: 8
                color: startColor
                border.color: "transparent"; border.width: 0
                Rectangle {
                    anchors.fill: parent
                    radius: parent.radius
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: actionBBtn.startColor }
                        GradientStop { position: 1.0; color: actionBBtn.endColor }
                    }
                }
                DashboardText {
                    anchors.centerIn: parent; text: actB
                    color: ioToggle.bActive ? cIoActiveText : "#ffffff"
                    font.pixelSize: 16; font.bold: true
                }
                MotionMouseArea {
                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    hoverScale: 1.012
                    pressScale: 0.99
                    shadowEnabled: false
                    shimmerEnabled: false
                    raiseOnHover: true
                    onPressed: actionBBtn.held = true
                    onReleased: actionBBtn.held = false
                    onCanceled: actionBBtn.held = false
                    onEntered: actionBBtn.hovered = true
                    onExited: { actionBBtn.hovered = false; actionBBtn.held = false }
                    onClicked: hpController.publishManual(ioId, actB)
                }
            }
        }
    }

    // Press-and-hold jog button: publishes "rev"/"fwd" on press, "stop" on release
    component JogBtn: Rectangle {
        id: jogBtn
        property string lbl: "JOG"
        property string dir: "fwd"  // "fwd" | "rev"
        property string variant: "default"
        property bool _pressed: false
        readonly property color baseBorder: variant === "primary" ? cAccent
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cBorder
        readonly property color gradStart:  variant === "primary" ? cBtnActionStart
                                          : variant === "warn"    ? cBtnWarnStart
                                          : variant === "danger"  ? cBtnDangerStart : cBtnBaseStart
        readonly property color gradEnd:    variant === "primary" ? cBtnActionEnd
                                          : variant === "warn"    ? cBtnWarnEnd
                                          : variant === "danger"  ? cBtnDangerEnd : cBtnBaseEnd
        readonly property color baseFg:     "#ffffff"
        implicitWidth: 140; implicitHeight: 46
        radius: 8
        color: _pressed ? Qt.darker(gradStart, 1.18) : gradStart
        border.color: "transparent"; border.width: 0
        Rectangle {
            anchors.fill: parent
            radius: parent.radius
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.0; color: jogBtn._pressed ? Qt.darker(jogBtn.gradStart, 1.18) : jogBtn.gradStart }
                GradientStop { position: 1.0; color: jogBtn._pressed ? Qt.darker(jogBtn.gradEnd, 1.18) : jogBtn.gradEnd }
            }
        }
        DashboardText {
            anchors.centerIn: parent; text: lbl; color: baseFg
            font.pixelSize: 21; font.bold: true
        }
        MotionMouseArea {
            anchors.fill: parent
            cursorShape: Qt.PointingHandCursor
            hoverScale: 1.012
            pressScale: 0.99
            shadowEnabled: false
            shimmerEnabled: false
            raiseOnHover: true
            onPressed:   { parent._pressed = true;  hpController.publishString("servo_jog", dir) }
            onReleased:  { parent._pressed = false; hpController.publishString("servo_jog", "stop") }
            onCanceled:  { parent._pressed = false; hpController.publishString("servo_jog", "stop") }
        }
    }

    // Numeric input rectangle (exposes valueText for parent to read)
    component PwmInput: Rectangle {
        property string valueText: "0"
        implicitWidth: 90; implicitHeight: 34
        radius: 4
        color: cField; border.color: cBorder; border.width: 1
        SmartTextInput {
            focusHost: tab.focusHost
            anchors.fill: parent; anchors.margins: 6
            text: valueText
            onTextChanged: valueText = text
            color: cText; font.pixelSize: 21; font.bold: true; font.family: monoFamily
            selectByMouse: true; horizontalAlignment: TextInput.AlignHCenter
        }
    }

    // Setting row (label + numeric input + unit + Set button)
    component SettingRow: Rectangle {
        property var item: ({})
        property string currentVal: ""
        height: 46
        radius: 6
        color: cPanel2; border.color: cBorder; border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10; anchors.rightMargin: 10
            spacing: 6
            DashboardText {
                text: item.label || ""; color: cText
                font.pixelSize: 20; font.bold: true
                Layout.fillWidth: true; elide: Text.ElideRight
            }
            Rectangle {
                implicitWidth: 95; implicitHeight: 32; radius: 4
                color: cField; border.color: cBorder; border.width: 1
                SmartTextInput {
                    id: inp
                    focusHost: tab.focusHost
                    anchors.fill: parent; anchors.margins: 6
                    text: currentVal
                    color: cText; font.pixelSize: 20; font.family: monoFamily
                    selectByMouse: true; horizontalAlignment: TextInput.AlignRight
                }
            }
            DashboardText { text: item.unit || ""; color: cMuted; font.pixelSize: 19; Layout.preferredWidth: 38 }
            Rectangle {
                id: setBtn
                property bool held: false
                property bool hovered: false
                readonly property color startColor: held ? cBtnActionPressStart : (hovered ? cBtnActionHoverStart : cBtnActionStart)
                readonly property color endColor: held ? cBtnActionPressEnd : (hovered ? cBtnActionHoverEnd : cBtnActionEnd)
                implicitWidth: 60; implicitHeight: 32; radius: 8
                color: startColor; border.color: "transparent"; border.width: 0
                Rectangle {
                    anchors.fill: parent
                    radius: parent.radius
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: setBtn.startColor }
                        GradientStop { position: 1.0; color: setBtn.endColor }
                    }
                }
                DashboardText {
                    anchors.centerIn: parent; text: "Set"; color: "#ffffff"
                    font.pixelSize: 19; font.bold: true
                }
                MotionMouseArea {
                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    hoverScale: 1.012
                    pressScale: 0.99
                    shadowEnabled: false
                    shimmerEnabled: false
                    raiseOnHover: true
                    onPressed: setBtn.held = true
                    onReleased: setBtn.held = false
                    onCanceled: setBtn.held = false
                    onEntered: setBtn.hovered = true
                    onExited: { setBtn.hovered = false; setBtn.held = false }
                    onClicked: tab.publishSetting(item, inp.text)
                }
            }
        }
    }
}
