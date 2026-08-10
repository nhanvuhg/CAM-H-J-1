import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15
import QtQuick.Window 2.15

ApplicationWindow {
    id: mainWindow
    visible: true
    width: 1920
    height: 1080
    title: qsTr("ROS2 - Unified Control System")
    color: "#020817"
    visibility: Window.FullScreen

    property bool scaleIssueWarning: false
    property bool autoAiStartedSinceModeSelect: false
    property string selectedCartridgeMode: cartridgeController.currentMode || "idle"
    property bool state2OutputFullWaitSeen: false

    onSelectedCartridgeModeChanged:
        systemAlertController.setOperationMode(cartridgeModeFor(selectedCartridgeMode))

    // Fill HP node-compatible authentication.
    property bool loginOpen: false
    property bool passwordVisible: false
    property string authRole: (authController.role || "").toString().trim().toLowerCase()
    property bool isAdmin: authController.authenticated && authRole === "admin"
    property bool isOperator: authController.authenticated && authRole === "operator"
    property bool isViewer: authController.authenticated && authRole === "viewer"
    property bool controlUnlocked: isAdmin || isOperator
    property bool readOnlyView: !loginOpen && ((!authController.authenticated) || (isViewer && stackView.depth <= 1))

    signal synchronizedModeRequested(string mode)
    signal synchronizedStartRequested(string mode)
    signal synchronizedStopRequested()

    function cartridgeModeFor(mode) {
        var m = (mode || "").toString().trim().toLowerCase()
        if (m === "auto")
            return "auto"
        if (m === "ai" || m === "camera_ai")
            return "ai"
        if (m === "jog")
            return "jog"
        return "manual"
    }

    function robotModeFor(mode) {
        var m = (mode || "").toString().trim().toLowerCase()
        if (m === "camera_ai" || m === "ai")
            return "ai"
        if (m === "auto")
            return "auto"
        return "manual"
    }

    function cartridgeCommandModeFor(mode) {
        var m = (mode || "").toString().trim().toLowerCase()
        return cartridgeModeFor(m)
    }

    function syncOperationMode(mode) {
        var cartridgeMode = cartridgeModeFor(mode)
        var cartridgeCommandMode = cartridgeCommandModeFor(mode)
        var robotMode = robotModeFor(mode)

        systemAlertController.setOperationMode(cartridgeMode)
        selectedCartridgeMode = cartridgeMode
        synchronizedModeRequested(mode)
        cartridgeController.setMode(cartridgeCommandMode)

        if (robotMode === "ai")
            robotController.setAiMode(true)
        else if (robotMode === "auto")
            robotController.setAutoMode(true)
        else
            robotController.setManualMode(true)

        return cartridgeMode
    }

    function startSynchronizedSystems(mode) {
        var requestedCartridgeMode = cartridgeModeFor(mode)
        if (!systemAlertController.prepareStart(requestedCartridgeMode))
            return false

        var cartridgeMode = syncOperationMode(mode)
        autoAiStartedSinceModeSelect = (cartridgeMode === "auto" || cartridgeMode === "ai")
        hpController.publishMode((cartridgeMode === "auto" || cartridgeMode === "ai") ? 0 : 2)
        synchronizedStartRequested(mode)
        robotController.startSystem(true)
        return true
    }

    function stopSynchronizedSystems() {
        synchronizedStopRequested()
        // Broadcast first. stopSystem() publishes /system/stop_button, which is
        // the shortest path to every abort: motion_executor sets abort_motion_
        // straight from it, robot_logic_node cancels its goals, the cartridge
        // node broadcasts servo STOP telegrams and the VFD drops the belt. The
        // service calls below are asynchronous, so nothing is delayed by
        // sending this one message ahead of them.
        cartridgeController.stopSystem()
        // STOP must always abort the active motion and clear its queued
        // command.  The Robot Control tab is also used in manual mode, where
        // the previous soft-stop path could leave a paused driver command
        // that resumed after ENABLE.
        robotController.stopAndResetRobot()
        autoAiStartedSinceModeSelect = false
        selectedCartridgeMode = "manual"
    }

    function emergencyStopSynchronizedSystems() {
        synchronizedStopRequested()
        // Same ordering rule as stopSynchronizedSystems: the broadcast reaches
        // the motion abort flag and the servo STOP telegrams, so it goes out
        // before the Dobot emergency-stop service call.
        cartridgeController.stopSystem()
        autoAiStartedSinceModeSelect = false
        selectedCartridgeMode = "manual"
        robotController.emergencyStop(true)
    }

    function showCameraPageOnly() {
        while (stackView.depth > 1)
            stackView.pop()
    }

    function openLoginDialog() {
        loginOpen = true
        passwordVisible = false
        loginPopup.open()
    }

    function enterReadOnlyView() {
        loginOpen = false
        passwordVisible = false
        loginPassword.clear()
        loginPopup.close()
        showCameraPageOnly()
    }

    onReadOnlyViewChanged: {
        if (readOnlyView)
            showCameraPageOnly()
    }

    Shortcut {
        sequence: "F11"
        onActivated: {
            if (mainWindow.visibility === Window.FullScreen)
                mainWindow.visibility = Window.Windowed
            else
                mainWindow.visibility = Window.FullScreen
        }
    }

    Item {
        id: darkNavyBackground
        anchors.fill: parent
        z: -10

        Rectangle {
            anchors.fill: parent
            gradient: Gradient {
                orientation: Gradient.Vertical
                GradientStop { position: 0.00; color: "#020715" }
                GradientStop { position: 0.42; color: "#06234c" }
                GradientStop { position: 0.72; color: "#041a38" }
                GradientStop { position: 1.00; color: "#01040d" }
            }
        }

        Rectangle {
            anchors.fill: parent
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.00; color: "#d9000309" }
                GradientStop { position: 0.22; color: "#3308203f" }
                GradientStop { position: 0.52; color: "#1f0b3560" }
                GradientStop { position: 0.78; color: "#3308203f" }
                GradientStop { position: 1.00; color: "#df000309" }
            }
        }

        Canvas {
            id: navyTexture
            anchors.fill: parent
            opacity: 0.22
            onWidthChanged: requestPaint()
            onHeightChanged: requestPaint()
            onPaint: {
                var ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)
                var dots = Math.max(2400, Math.floor(width * height / 620))
                for (var i = 0; i < dots; ++i) {
                    var x = Math.random() * width
                    var y = Math.random() * height
                    var a = 0.018 + Math.random() * 0.055
                    var s = 0.5 + Math.random() * 1.8
                    ctx.fillStyle = "rgba(120,170,220," + a + ")"
                    ctx.fillRect(x, y, s, s)
                }
                for (var j = 0; j < 120; ++j) {
                    ctx.strokeStyle = "rgba(120,160,210,0.025)"
                    ctx.lineWidth = 1
                    ctx.beginPath()
                    ctx.moveTo(Math.random() * width, Math.random() * height)
                    ctx.lineTo(Math.random() * width, Math.random() * height)
                    ctx.stroke()
                }
            }
        }

        Rectangle {
            anchors.fill: parent
            color: "#30000000"
        }
    }

    StackView {
        id: stackView
        anchors.fill: parent
        initialItem: cameraPage

        Component {
            id: cameraPage
            CameraPage {}
        }
        Component {
            id: cartridgePage
            CartridgePage {}
        }
    }

    // ────────────────────────────────────────────────────────────
    // GLOBAL POPUP — State 2 blocked because the output tray stack is full
    // ACCEPT only requests a fresh S4 check. The backend owns the state and
    // closes this popup by reporting s2_output_clear or leaving the wait state.
    Connections {
        target: cartridgeController

        function onNotificationReceived() {
            var obj
            try {
                obj = JSON.parse(cartridgeController.lastNotification || "{}")
            } catch (error) {
                return
            }

            var code = (obj.code || "").toString().trim().toLowerCase()
            if (code === "s2_output_full") {
                state2OutputFullPopup.open()
            } else if (code === "s2_output_clear") {
                mainWindow.state2OutputFullWaitSeen = false
                state2OutputFullPopup.close()
            }
        }

        function onSystemStateChanged() {
            var state = (cartridgeController.stateIn || "").toString().trim().toLowerCase()
            var waitingForOutputClear = state === "s2a_wait_output_clear"

            if (waitingForOutputClear) {
                mainWindow.state2OutputFullWaitSeen = true
                state2OutputFullPopup.open()
            } else if (mainWindow.state2OutputFullWaitSeen) {
                mainWindow.state2OutputFullWaitSeen = false
                state2OutputFullPopup.close()
            }
        }
    }

    Popup {
        id: state2OutputFullPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 680
        height: 390

        background: Rectangle {
            color: "#081627"
            border.color: "#f0735c"
            border.width: 3
            radius: 10
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 24
            spacing: 16

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: qsTr("⚠  OUTPUT TRAY FULL")
                color: "#f0735c"
                font.pixelSize: 28
                font.bold: true
            }

            Text {
                Layout.fillWidth: true
                text: qsTr("Sensor S4 is ON when STATE 2 starts: the output tray is full.\n" +
                           "The system is holding InX at -60 mm and InY at 87 mm to check the tray.\n" +
                           "InX will not move to the 502.5 mm tray pickup position.\n\n" +
                           "Remove the output tray, then select ACCEPT to recheck S4.")
                color: "#c7dcef"
                font.pixelSize: 18
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }

            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 18

                MotionButton {
                    Layout.preferredWidth: 360
                    Layout.preferredHeight: 60
                    text: qsTr("✓  ACCEPT — RECHECK S4")
                    font.pixelSize: 17
                    font.bold: true
                    background: Rectangle {
                        color: "#0a493b"
                        border.color: "#3ed0b4"
                        border.width: 2
                        radius: 6
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#d9fff7"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: cartridgeController.acceptState2OutputFull()
                }

                MotionButton {
                    Layout.preferredWidth: 220
                    Layout.preferredHeight: 60
                    text: qsTr("⏹  STOP")
                    font.pixelSize: 17
                    font.bold: true
                    background: Rectangle {
                        color: "#3a1614"
                        border.color: "#f0735c"
                        border.width: 2
                        radius: 6
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#f0735c"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: mainWindow.stopSynchronizedSystems()
                }
            }
        }
    }

    // GLOBAL POPUP — feed_chamber timeout resume choice
    // ────────────────────────────────────────────────────────────
    // GLOBAL POPUP — feed_chamber timeout resume choice
    // Robot_logic_node publishes systemStatus = "WAIT_RESUME_CHOICE" when
    // LOAD_CHAMBER_FROM_BUFFER times out (150s) + SCALE has been drained to
    // PLACE. Operator chooses 1 of 2 ways to resume or stops the system.
    // Popup is displayed globally (on all pages) so operator does not miss it.
    // ────────────────────────────────────────────────────────────
    Connections {
        target: robotController
        function onSystemStatusChanged() {
            var s = robotController.systemStatus
            if (s === "WAIT_RESUME_CHOICE") {
                scaleChoicePopup.close()
                confirmEmptyBufferPopup.close()
                resumeChoicePopup.open()
            } else if (s === "WAIT_SCALE_CHOICE") {
                resumeChoicePopup.close()
                confirmEmptyBufferPopup.close()
                scaleChoicePopup.open()
            } else {
                resumeChoicePopup.close()
                confirmEmptyBufferPopup.close()
                scaleChoicePopup.close()
            }
        }
    }

    Popup {
        id: resumeChoicePopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 620; height: 400
        background: Rectangle {
            color: "#081627"
            border.color: "#f5a623"
            border.width: 2
            radius: 10
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: qsTr("⏸  RESUME REQUIRED")
                color: "#f5a623"
                font.pixelSize: 26
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: qsTr("feed_chamber timed out after 150 seconds — SCALE has been drained to PLACE.\nSelect how to resume the cycle:")
                color: "#c7dcef"
                font.pixelSize: 17
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 16

                MotionButton {
                    Layout.preferredWidth: 250; Layout.preferredHeight: 60
                    text: qsTr("🔁  LOAD CHAMBER\nFROM BUFFER")
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle {
                        radius: 6
                        border.color: "#9b7bff"; border.width: 2
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: "#6f4be0" }
                            GradientStop { position: 1.0; color: "#452e91" }
                        }
                    }
                    contentItem: Text {
                        text: parent.text; color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("LOAD_CHAMBER_FROM_BUFFER")
                        resumeChoicePopup.close()
                    }
                }
                MotionButton {
                    Layout.preferredWidth: 250; Layout.preferredHeight: 60
                    text: qsTr("🔂  LOAD CHAMBER\nFROM TRAY")
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle {
                        radius: 6
                        border.color: "#f5a623"; border.width: 2
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: "#e2761b" }
                            GradientStop { position: 1.0; color: "#8a4210" }
                        }
                    }
                    contentItem: Text {
                        text: parent.text; color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: confirmEmptyBufferPopup.open()
                }
            }

            MotionButton {
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 516; Layout.preferredHeight: 52
                text: qsTr("⏹  STOP — Stop the system and hold the current position")
                font.pixelSize: 15; font.bold: true
                background: Rectangle { color: "#3a1614"; border.color: "#f0735c"; border.width: 2; radius: 6 }
                contentItem: Text {
                    text: parent.text; color: "#f0735c"
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    mainWindow.stopSynchronizedSystems()
                    resumeChoicePopup.close()
                }
            }
        }
    }

    Popup {
        id: confirmEmptyBufferPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 580; height: 320
        background: Rectangle {
            color: "#081627"
            border.color: "#f0735c"
            border.width: 2
            radius: 10
        }
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: qsTr("⚠  CONFIRM EMPTY BUFFER")
                color: "#f0735c"
                font.pixelSize: 24
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: qsTr("LOAD CHAMBER FROM TRAY will restart like a fresh boot:\n" +
                           "  INIT_LOAD → INIT_REFILL_BUFFER → cycle.\n\n" +
                           "Have you manually removed all cartridges from the BUFFER?")
                color: "#c7dcef"
                font.pixelSize: 16
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 16

                MotionButton {
                    Layout.preferredWidth: 240; Layout.preferredHeight: 56
                    text: qsTr("✓  Buffer is empty — CONFIRM")
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#3ed0b4"; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#04140d"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("INIT_LOAD_CHAMBER_DIRECT")
                        confirmEmptyBufferPopup.close()
                        resumeChoicePopup.close()
                    }
                }
                MotionButton {
                    Layout.preferredWidth: 240; Layout.preferredHeight: 56
                    text: qsTr("✗  Cancel / Back")
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#14263c"; border.color: "#1a4a6e"; border.width: 1; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#c7dcef"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: confirmEmptyBufferPopup.close()
                }
            }
        }
    }

    // ────────────────────────────────────────────────────────────
    // GLOBAL POPUP — loadcell silent 150s in PROCESSING_SCALE
    // Operator chọn 1 trong 3 cách xử lý: WAIT_FILLING / PLACE_OUTPUT / PLACE_FAIL
    // ────────────────────────────────────────────────────────────
    Popup {
        id: scaleChoicePopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 660; height: 430
        background: Rectangle {
            color: "#081627"
            border.color: "#f0735c"
            border.width: 2
            radius: 10
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: qsTr("⚠  SCALE ISSUE DETECTED")
                color: "#f0735c"
                font.pixelSize: 26
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: qsTr("No loadcell topic was received for 150 seconds in PROCESSING_SCALE.\nSelect how to handle this cartridge:")
                color: "#c7dcef"
                font.pixelSize: 16
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 12

                MotionButton {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: qsTr("↩  BACK TO\nWAIT FILLING\n(cartridge removed)")
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#081627"; border.color: "#36b6ff"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#7fcdf5"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("WAIT_FILLING")
                        mainWindow.scaleIssueWarning = true
                        scaleChoicePopup.close()
                    }
                }
                MotionButton {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: qsTr("✓  PLACE TO\nOUTPUT\n(force PASS)")
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#0a2418"; border.color: "#3ed0b4"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#3ed0b4"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("PLACE_TO_OUTPUT")
                        mainWindow.scaleIssueWarning = true
                        scaleChoicePopup.close()
                    }
                }
                MotionButton {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: qsTr("✗  PLACE TO\nFAIL\n(force FAIL)")
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#220c0b"; border.color: "#f0735c"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#f5a394"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("PLACE_TO_FAIL")
                        mainWindow.scaleIssueWarning = true
                        scaleChoicePopup.close()
                    }
                }
            }

            MotionButton {
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 612; Layout.preferredHeight: 52
                text: qsTr("⏹  STOP — Stop the system and hold the current position")
                font.pixelSize: 15; font.bold: true
                background: Rectangle { color: "#160a09"; border.color: "#f0735c"; border.width: 2; radius: 6 }
                contentItem: Text {
                    text: parent.text; color: "#f0735c"
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    mainWindow.stopSynchronizedSystems()
                    mainWindow.scaleIssueWarning = true
                    scaleChoicePopup.close()
                }
            }
        }
    }

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

    function checkInkAndRun(callback) {
        var inkMap = parseKvPipe(hpController.inkStatus);
        var code = inkMap["CODE"] || "";
        var lot_ci = inkMap["LOT_CI"] || "";
        
        var sysMap = parseKvPipe(hpController.systemStatus);
        var modeStr = (sysMap["MODE"] || hpController.modeStatus || "").toString().trim().toUpperCase();

        var isAutoOrPrefill = (modeStr === "AUTO" || modeStr === "PREFILL" || modeStr === "1" || modeStr === "2");
        var isInkEmpty = (code.trim() === "" || lot_ci.trim() === "");

        if (isAutoOrPrefill && isInkEmpty) {
            notYetInkSelectedPopup.confirmCallback = callback;
            notYetInkSelectedPopup.open();
        } else {
            callback();
        }
    }

    Popup {
        id: notYetInkSelectedPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 600; height: 320
        background: Rectangle {
            color: "#06101d"
            border.color: "#f0735c"
            border.width: 2
            radius: 10
        }
        
        property var confirmCallback: null
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 25
            spacing: 15
            
            Text {
                text: qsTr("⚠️ WARNING: INK NOT SELECTED")
                color: "#f0735c"
                font.pixelSize: 24
                font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: qsTr("Ink or Lot has not been selected for the system.\nIf you continue running, production and consumption logs WILL NOT BE SAVED.")
                color: "#c7dcef"
                font.pixelSize: 16
                Layout.alignment: Qt.AlignHCenter
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
            Item { Layout.fillHeight: true }
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 20
                MotionButton {
                    text: qsTr("✓ RUN (NO LOGGING)")
                    font.pixelSize: 16
                    font.bold: true
                    onClicked: {
                        notYetInkSelectedPopup.close()
                        if (notYetInkSelectedPopup.confirmCallback) notYetInkSelectedPopup.confirmCallback()
                    }
                    background: Rectangle { radius: 6; color: "#f0735c" }
                    contentItem: Text {
                        text: parent.text; color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                MotionButton {
                    text: qsTr("✗ CANCEL")
                    font.pixelSize: 16
                    font.bold: true
                    onClicked: notYetInkSelectedPopup.close()
                    background: Rectangle { radius: 6; color: "#14263c"; border.color: "#1a4a6e"; border.width: 1 }
                    contentItem: Text {
                        text: parent.text; color: "#c7dcef"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }
    }

    Item {
        id: readOnlyShield
        anchors.fill: parent
        visible: mainWindow.readOnlyView
        z: 800

        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.AllButtons
            hoverEnabled: true
            preventStealing: true

            function isCameraCloseButtonArea(px, py) {
                return px >= width - 76 && px <= width - 6
                        && py >= 6 && py <= 86
            }

            function isCameraCartridgeButtonArea(px, py) {
                return mainWindow.isViewer
                        && px >= width - 136 && px <= width - 76
                        && py >= 6 && py <= 86
            }

            function shouldPassThrough(px, py) {
                return isCameraCloseButtonArea(px, py) || isCameraCartridgeButtonArea(px, py)
            }

            onClicked: mouse.accepted = !shouldPassThrough(mouse.x, mouse.y)
            onPressed: mouse.accepted = !shouldPassThrough(mouse.x, mouse.y)
            onReleased: mouse.accepted = !shouldPassThrough(mouse.x, mouse.y)
            onWheel: wheel.accepted = !shouldPassThrough(wheel.x, wheel.y)
        }

        Rectangle {
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.topMargin: 96
            anchors.rightMargin: 150
            width: 310
            height: 58
            radius: 8
            color: "#cc06101d"
            border.color: "#3ed0b4"
            border.width: 1
            z: 2

            RowLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 10

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 0

                    Text {
                        Layout.fillWidth: true
                        text: authController.authenticated ? qsTr("VIEWER MODE") : qsTr("VIEW ONLY")
                        color: "#7fcdf5"
                        font.pixelSize: 15
                        font.bold: true
                        elide: Text.ElideRight
                    }

                    Text {
                        Layout.fillWidth: true
                        text: qsTr("Camera page only")
                        color: "#9fb3c8"
                        font.pixelSize: 12
                        elide: Text.ElideRight
                    }
                }

                MotionButton {
                    Layout.preferredWidth: 92
                    Layout.preferredHeight: 40
                    text: qsTr("LOGIN")
                    font.pixelSize: 14
                    font.bold: true
                    hoverScale: 1.02
                    pressScale: 0.98
                    onClicked: mainWindow.openLoginDialog()
                    background: Rectangle {
                        radius: 6
                        color: "#14263c"
                        border.color: "#3ed0b4"
                        border.width: 1
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#d6f1ff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }
    }

    Popup {
        id: loginPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: 520
        // Tự co theo nội dung: fix cứng height 430 làm nút LOGIN tràn khỏi khung
        // (Popup mặc định còn có padding 12 mỗi cạnh)
        height: loginColumn.implicitHeight + 2 * loginColumn.anchors.margins
                + topPadding + bottomPadding
        modal: true
        focus: true
        closePolicy: Popup.NoAutoClose

        background: Rectangle {
            radius: 12
            color: "#06101d"
            border.color: "#3ed0b4"
            border.width: 2

            gradient: Gradient {
                orientation: Gradient.Vertical
                GradientStop { position: 0.0; color: "#0d1e32" }
                GradientStop { position: 0.55; color: "#081627" }
                GradientStop { position: 1.0; color: "#06101d" }
            }

            Rectangle {
                anchors.fill: parent
                anchors.margins: 1
                radius: 11
                color: "transparent"
                border.color: "#22ffffff"
                border.width: 1
            }
        }

        Item {
            anchors.fill: parent

            MotionButton {
                id: closeLoginButton
                anchors.top: parent.top
                anchors.right: parent.right
                anchors.topMargin: 14
                anchors.rightMargin: 14
                width: 42
                height: 42
                text: "X"
                font.pixelSize: 18
                font.bold: true
                hoverScale: 1.04
                pressScale: 0.96
                onClicked: mainWindow.enterReadOnlyView()
                background: Rectangle {
                    radius: 6
                    color: closeLoginButton.pressed ? "#7a2424" : "#14263c"
                    border.color: closeLoginButton.hovered ? "#f0735c" : "#1a4a6e"
                    border.width: 1
                }
                contentItem: Text {
                    text: closeLoginButton.text
                    color: closeLoginButton.hovered ? "#ffffff" : "#c7dcef"
                    font: closeLoginButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }

            LanguageSelector {
                id: loginLanguageSelector
                anchors.top: parent.top
                anchors.right: closeLoginButton.left
                anchors.topMargin: 14
                anchors.rightMargin: 8
                width: 42
                height: 42
                panelColor: "#14263c"
                panelColorDeep: "#06101d"
                borderColor: "#1a4a6e"
                textColor: "#d6f1ff"
                mutedColor: "#74899f"
                accentColor: "#7fcdf5"
            }

            ColumnLayout {
                id: loginColumn
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.margins: 30
                anchors.rightMargin: 34
                spacing: 12

                Text {
                    Layout.fillWidth: true
                    Layout.rightMargin: 94
                    text: qsTr("SYSTEM LOGIN")
                    color: "#7fcdf5"
                    font.pixelSize: 24
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                }

                Text {
                    Layout.fillWidth: true
                    Layout.rightMargin: 94
                    text: qsTr("Use a Fill HP account to unlock controls")
                    color: "#9fb3c8"
                    font.pixelSize: 15
                    horizontalAlignment: Text.AlignHCenter
                }

                Item { Layout.preferredHeight: 4 }

                Text {
                    Layout.fillWidth: true
                    text: qsTr("Account")
                    color: "#d6f1ff"
                    font.pixelSize: 15
                    font.bold: true
                }

                TextField {
                    id: loginUsername
                    Layout.fillWidth: true
                    Layout.preferredHeight: 52
                    placeholderText: qsTr("Enter account")
                    color: "#ffffff"
                    placeholderTextColor: "#6f8ba4"
                    selectByMouse: true
                    font.pixelSize: 18
                    onAccepted: loginPassword.forceActiveFocus()
                    background: Rectangle {
                        radius: 7
                        color: "#0c1726"
                        border.color: loginUsername.activeFocus ? "#3ed0b4" : "#163a52"
                        border.width: loginUsername.activeFocus ? 2 : 1
                    }
                }

                Text {
                    Layout.fillWidth: true
                    text: qsTr("Password")
                    color: "#d6f1ff"
                    font.pixelSize: 15
                    font.bold: true
                }

                TextField {
                    id: loginPassword
                    Layout.fillWidth: true
                    Layout.preferredHeight: 52
                    rightPadding: 58
                    placeholderText: qsTr("Enter password")
                    color: "#ffffff"
                    placeholderTextColor: "#6f8ba4"
                    echoMode: mainWindow.passwordVisible ? TextInput.Normal : TextInput.Password
                    selectByMouse: true
                    font.pixelSize: 18
                    onAccepted: loginButton.clicked()
                    background: Rectangle {
                        radius: 7
                        color: "#0c1726"
                        border.color: loginPassword.activeFocus ? "#3ed0b4" : "#163a52"
                        border.width: loginPassword.activeFocus ? 2 : 1
                    }

                    ToolButton {
                        id: passwordEyeButton
                        anchors.right: parent.right
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.rightMargin: 6
                        width: 44
                        height: 40
                        hoverEnabled: true
                        onClicked: mainWindow.passwordVisible = !mainWindow.passwordVisible
                        background: Rectangle {
                            radius: 6
                            color: passwordEyeButton.hovered || mainWindow.passwordVisible ? "#163a52" : "transparent"
                            border.color: passwordEyeButton.hovered ? "#3ed0b4" : "transparent"
                            border.width: 1
                        }
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: "qrc:/qml/icons/eye.svg"
                                width: 26
                                height: 26
                                fillMode: Image.PreserveAspectFit
                                opacity: mainWindow.passwordVisible ? 1.0 : 0.7
                            }
                        }
                    }
                }

                Text {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 22
                    text: authController.lastError
                    color: "#f0735c"
                    font.pixelSize: 14
                    horizontalAlignment: Text.AlignHCenter
                    // Luôn giữ chỗ để popup không nhảy khi có/hết lỗi
                }

                Item { Layout.preferredHeight: 6 }

                MotionButton {
                    id: loginButton
                    Layout.fillWidth: true
                    Layout.preferredHeight: 56
                    text: qsTr("LOGIN")
                    font.pixelSize: 18
                    font.bold: true
                    hoverScale: 1.01
                    pressScale: 0.985
                    onClicked: {
                        if (authController.login(loginUsername.text, loginPassword.text)) {
                            mainWindow.loginOpen = false
                            mainWindow.passwordVisible = false
                            loginPassword.clear()
                            loginPopup.close()
                            mainWindow.showCameraPageOnly()
                        }
                    }
                    background: Rectangle {
                        radius: 7
                        border.color: "#3ed0b4"
                        border.width: 1
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: loginButton.pressed ? "#177c6d" : "#1f9e86" }
                            GradientStop { position: 1.0; color: loginButton.pressed ? "#102739" : "#163a52" }
                        }
                    }
                    contentItem: Text {
                        text: loginButton.text
                        color: "#ffffff"
                        font: loginButton.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }

        onOpened: {
            mainWindow.loginOpen = true
            loginUsername.forceActiveFocus()
        }
        onClosed: mainWindow.loginOpen = false
        Component.onCompleted: {
            if (!authController.authenticated)
                mainWindow.openLoginDialog()
        }
    }

    Connections {
        target: authController
        function onAuthenticatedChanged() {
            if (authController.authenticated) {
                mainWindow.loginOpen = false
                mainWindow.passwordVisible = false
                loginPopup.close()
                mainWindow.showCameraPageOnly()
            } else {
                loginUsername.clear()
                loginPassword.clear()
                mainWindow.openLoginDialog()
            }
        }
    }
}
