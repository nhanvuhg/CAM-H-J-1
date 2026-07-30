import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Item {
    id: cameraPageRoot

    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    property bool rowLocked: (robotController.systemStatus || "").toUpperCase() === "INIT_LOAD_CHAMBER_DIRECT" && robotController.selectedRow >= 1
    property bool modeLocked: false
    property bool startCommandLocked: false
    property bool autoRowIndicatorsActive: false
    property string ctrlMode: "auto"  // "auto" | "camera_ai"
    property string pendingStartMode: ""
    property string pendingStartUiMode: ""
    property bool pauseLatched: false

    readonly property color cPanel:       "#990d1e32"
    readonly property color cPanel2:      "#8806101d"
    readonly property color cBorder:      "#1affffff"
    readonly property color cHover:       "#40ffffff"
    readonly property color cText:        "#ffffff"
    readonly property color cMuted:       "#9fb3c8"
    readonly property color cAccent:      "#7fcdf5"
    readonly property color cOk:          "#3ed0b4"
    readonly property color cOkBg:        Qt.rgba(0.0, 0.90, 0.46, 0.15)
    readonly property color cWarn:        "#f5a623"
    readonly property color cWarnBg:      Qt.rgba(1.0, 0.65, 0.15, 0.15)
    readonly property color cBad:         "#f0735c"
    readonly property color cBadBg:       Qt.rgba(1.0, 0.32, 0.32, 0.15)
    // CameraPage liquid-glass action palette.
    readonly property color cBtnBaseStart:      "#0c1726"
    readonly property color cBtnBaseEnd:        "#06101d"
    readonly property color cBtnBaseBorder:     "#163a52"
    readonly property color cBtnBaseText:       "#d6f1ff"
    readonly property color cBtnPrimaryStart:   "#1f9e86"
    readonly property color cBtnPrimaryEnd:     "#163a52"
    readonly property color cBtnPrimaryBorder:  "#3ed0b4"
    readonly property color cBtnActionStart:    "#163a52"
    readonly property color cBtnActionEnd:      "#081627"
    readonly property color cBtnActionBorder:   "#163a52"
    readonly property color cBtnClearStart:     "#234C6A"
    readonly property color cBtnClearEnd:       "#102739"
    readonly property color cBtnClearText:      "#ffffff"
    readonly property color cBtnDangerStart:    "#E05454"
    readonly property color cBtnDangerEnd:      "#7a2424"
    readonly property color cBtnDangerBorder:   "#E05454"
    readonly property color cBtnEmergencyStart: "#E05454"
    readonly property color cBtnEmergencyEnd:   "#9c3030"
    readonly property color cBtnEmergencyBorder:"#E05454"
    readonly property color cBtnWarningStart:   "#e2761b"
    readonly property color cBtnWarningEnd:     "#8a4210"
    readonly property color cBtnWarningBorder:  "#f5a623"
    readonly property color cServoJogStart:     "#A1C2BD"
    readonly property color cServoJogEnd:       "#163a52"
    readonly property color cServoJogBorder:    "#163a52"
    readonly property color cServoJogText:      "#06101d"
    readonly property color cServoRunStart:     "#1C4D8D"
    readonly property color cServoRunEnd:       "#0c1726"
    readonly property color cServoRunBorder:    "#163a52"
    readonly property color cServoRunText:      "#ffffff"

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
        if (!list || list.length === 0) {
            return "Min 0.0   Avg 0.0   Max\n0.0 mbar";
        }
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

    // Calculated from systemStatus - block mode change when robot is running
    // MANUAL is considered "idle" (not busy) - robot only waits for manual commands, allowing mode change.
    property bool robotBusy: {
        var s = (robotController.systemStatus || "").toLowerCase()
        return s !== "" && s !== "idle" && s !== "ready" && s !== "unknown" && s !== "manual"
    }

    function dispatchStartAfterModeConfirmed() {
        var requestedUiMode = pendingStartUiMode
        if (requestedUiMode === "")
            return

        pendingStartMode = ""
        pendingStartUiMode = ""
        startModeConfirmTimer.stop()

        if (requestedUiMode === "camera_ai") {
            robotController.selectRow(0)
        }

        var lockControlMode = requestedUiMode === "auto" || requestedUiMode === "camera_ai"
        modeLocked = lockControlMode
        autoRowIndicatorsActive = lockControlMode
        mainWindow.startSynchronizedSystems(requestedUiMode)
    }

    // Unlock row selection khi process idle/done
    Connections {
        target: robotController
        function onSystemStatusChanged() {
            var s = (robotController.systemStatus || "").toLowerCase()
            if (s === "idle" || s === "ready" || s === "") {
                cameraPageRoot.modeLocked = false
            }
        }
    }

    Connections {
        target: mainWindow
        function onSynchronizedModeRequested(mode) {
            var m = (mode || "").toLowerCase()
            if (m === "auto" || m === "camera_ai" || m === "manual")
                cameraPageRoot.ctrlMode = m
            else if (m === "ai")
                cameraPageRoot.ctrlMode = "camera_ai"
            else if (m === "jog")
                cameraPageRoot.ctrlMode = "manual"
        }
        function onSynchronizedStartRequested(mode) {
            var m = (mode || "").toLowerCase()
            if (m === "auto" || m === "camera_ai" || m === "ai") {
                cameraPageRoot.modeLocked = true
                cameraPageRoot.autoRowIndicatorsActive = true
            }
        }
        function onSynchronizedStopRequested() {
            startModeConfirmTimer.stop()
            cameraPageRoot.pendingStartMode = ""
            cameraPageRoot.pendingStartUiMode = ""
            cameraPageRoot.modeLocked = false
            cameraPageRoot.startCommandLocked = false
            cameraPageRoot.autoRowIndicatorsActive = false
            cameraPageRoot.pauseLatched = false
        }
    }

    // Sync mode from Cartridge (ensures both UIs are synchronized)
    Connections {
        target: cartridgeController
        function onCurrentModeChanged() {
            var m = (cartridgeController.currentMode || "").toLowerCase()
            if (cameraPageRoot.pendingStartMode !== "" && m === cameraPageRoot.pendingStartMode)
                cameraPageRoot.dispatchStartAfterModeConfirmed()
            if (m === "auto" || m === "ai" || m === "camera_ai" || m === "manual" || m === "jog") {
                cameraPageRoot.ctrlMode = (m === "ai") ? "camera_ai" : (m === "jog" ? "manual" : m);
            }
        }
    }
    Timer {
        id: startModeConfirmTimer
        interval: 3000
        repeat: false
        onTriggered: {
            console.warn("START cancelled: cartridge mode confirmation timeout")
            cameraPageRoot.pendingStartMode = ""
            cameraPageRoot.pendingStartUiMode = ""
            cameraPageRoot.startCommandLocked = false
            cameraPageRoot.modeLocked = false
            cameraPageRoot.autoRowIndicatorsActive = false
        }
    }
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: currentTime = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    }

    // Ambient glow blobs — glass depth background
    Canvas {
        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            var g1 = ctx.createRadialGradient(width * 0.80, -height * 0.05, 0, width * 0.80, -height * 0.05, height * 0.85)
            g1.addColorStop(0.00, "rgba(20,90,210,0.40)")
            g1.addColorStop(0.30, "rgba(20,90,210,0.15)")
            g1.addColorStop(0.62, "rgba(20,90,210,0.03)")
            g1.addColorStop(1.00, "rgba(20,90,210,0.00)")
            ctx.fillStyle = g1; ctx.fillRect(0, 0, width, height)
            var g2 = ctx.createRadialGradient(width * 0.05, height * 1.10, 0, width * 0.05, height * 1.10, height * 0.65)
            g2.addColorStop(0.00, "rgba(0,175,155,0.30)")
            g2.addColorStop(0.40, "rgba(0,175,155,0.08)")
            g2.addColorStop(0.70, "rgba(0,175,155,0.02)")
            g2.addColorStop(1.00, "rgba(0,175,155,0.00)")
            ctx.fillStyle = g2; ctx.fillRect(0, 0, width, height)
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // ── Title Bar ──────────────────────────────────────────
        Item {
            Layout.fillWidth: true
            height: 80

            Rectangle {
                anchors.fill: parent
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8

                // specular highlight — top edge
                Rectangle {
                    anchors { top: parent.top; left: parent.left; right: parent.right }
                    height: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.35; color: "#55ffffff" }
                        GradientStop { position: 0.65; color: "#55ffffff" }
                        GradientStop { position: 1.0;  color: "transparent" }
                    }
                }

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10

                    ScreenshotButton {
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        onCaptureRequested: {
                            robotController.captureScreenshot()
                        }
                    }

                    MotionButton {
                        id: refreshNodesBtn
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        hoverScale: 1.012
                        pressScale: 0.99
                        onClicked: robotController.restartSystemNodes()
                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: cServoRunEnd
                            border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: refreshNodesBtn.pressed ? Qt.darker(cServoRunStart, 1.15) : cServoRunStart }
                                GradientStop { position: 1.0; color: refreshNodesBtn.pressed ? Qt.darker(cServoRunEnd, 1.15) : cServoRunEnd }
                            }
                        }
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: "qrc:/qml/icons/list_restart.svg"
                                width: 34; height: 34
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }
                            HoverHint {
                                visible: refreshNodesBtn.hovered
                                label: "Restart Node"
                                bc: cServoRunEnd
                                tc: cBtnBaseText
                            }
                        }
                    }

                    MotionButton {
                        id: restartGuiBtn
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        hoverScale: 1.012
                        pressScale: 0.99
                        onClicked: robotController.restartGui()
                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: cServoRunEnd
                            border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: restartGuiBtn.pressed ? Qt.darker(cServoRunStart, 1.15) : cServoRunStart }
                                GradientStop { position: 1.0; color: restartGuiBtn.pressed ? Qt.darker(cServoRunEnd, 1.15) : cServoRunEnd }
                            }
                        }
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: "qrc:/qml/icons/refresh_cw.svg"
                                width: 34; height: 34
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }
                            HoverHint {
                                visible: restartGuiBtn.hovered
                                label: "Restart GUI"
                                bc: cServoRunEnd
                                tc: cBtnBaseText
                            }
                        }
                    }

                    MotionButton {
                        id: ignoreScaleBtn
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        hoverScale: 1.012
                        pressScale: 0.99
                        onClicked: robotController.ignoreScale = !robotController.ignoreScale
                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: robotController.ignoreScale ? cBtnDangerEnd : cServoRunEnd
                            border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: ignoreScaleBtn.pressed ? Qt.darker(cServoRunStart, 1.15) : cServoRunStart }
                                GradientStop { position: 1.0; color: ignoreScaleBtn.pressed ? Qt.darker(cServoRunEnd, 1.15) : cServoRunEnd }
                            }
                            Behavior on border.color { ColorAnimation { duration: 140 } }
                        }
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: robotController.ignoreScale
                                        ? "qrc:/qml/icons/weight_tilde_lucide_red.svg"
                                        : "qrc:/qml/icons/weight_tilde_lucide.svg"
                                width: 34; height: 34
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }
                            HoverHint {
                                visible: ignoreScaleBtn.hovered
                                label: "Ignore Scale"
                                bc: robotController.ignoreScale ? cBtnDangerEnd : cServoRunEnd
                                tc: robotController.ignoreScale ? cBad : cBtnBaseText
                            }
                        }
                    }

                    MotionButton {
                        id: settingsBtn
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        hoverScale: 1.012
                        pressScale: 0.99
                        onClicked: {
                            var comp = Qt.createComponent("frm_settings.qml")
                            if (comp.status === Component.Ready) {
                                var win = comp.createObject(mainWindow)
                                if (win) {
                                    win.x = mainWindow.x + (mainWindow.width - win.width) / 2
                                    win.y = mainWindow.y + (mainWindow.height - win.height) / 2
                                    win.show()
                                }
                            }
                        }
                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: cServoRunEnd
                            border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: cServoRunStart }
                                GradientStop { position: 1.0; color: cServoRunEnd }
                            }
                        }
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: "icons/switch_camera.svg"
                                width: 34
                                height: 34
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }
                            HoverHint {
                                visible: settingsBtn.hovered
                                label: "Camera Setting"
                                bc: cServoRunEnd
                                tc: cBtnBaseText
                            }
                        }
                    }

                    Item {
                        Layout.fillWidth: true
                        Text {
                            anchors.centerIn: parent
                            text: "ROS 2 - INKOBOT MONITORING SYSTEM"
                            font.pixelSize: 24; font.bold: true; color: cAccent; font.letterSpacing: 2
                        }
                    }

                    MotionButton {
                        id: cartridgePageBtn
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        hoverScale: 1.012
                        pressScale: 0.99
                        onClicked: stackView.push(cartridgePage)
                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: cServoRunEnd
                            border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: cartridgePageBtn.pressed ? Qt.darker(cServoRunStart, 1.15) : cServoRunStart }
                                GradientStop { position: 1.0; color: cartridgePageBtn.pressed ? Qt.darker(cServoRunEnd, 1.15) : cServoRunEnd }
                            }
                        }
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: "qrc:/qml/icons/user_cog.svg"
                                width: 34; height: 34
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }
                            HoverHint {
                                visible: cartridgePageBtn.hovered
                                label: "Cartridge Page"
                                bc: cServoRunEnd
                                tc: cBtnBaseText
                            }
                        }
                    }

                    MotionButton {
                        id: closeGuiBtn
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        hoverScale: 1.012
                        pressScale: 0.99
                        onClicked: Qt.quit()
                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: cBtnDangerEnd
                            border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: cBtnDangerStart }
                                GradientStop { position: 1.0; color: cBtnDangerEnd }
                            }
                        }
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: "qrc:/icons/qml/icons/power_settings.svg"
                                width: 30; height: 30
                                fillMode: Image.PreserveAspectFit; smooth: true
                            }
                            HoverHint {
                                visible: closeGuiBtn.hovered
                                label: "Tắt giao diện"
                                bc: cBtnDangerEnd
                                tc: "#ffffff"
                            }
                        }
                    }
                }
            }
        }

        // ── Scale Issue Warning Banner ─────────────────────────
        Rectangle {
            id: scaleWarnBanner
            Layout.fillWidth: true
            height: 44
            visible: mainWindow.scaleIssueWarning
            color: "#240f0e"
            border.color: "#f0735c"
            border.width: 1
            radius: 4

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 16
                anchors.rightMargin: 10
                spacing: 12

                Text {
                    text: "⚠"
                    color: "#f0735c"
                    font.pixelSize: 20
                    font.bold: true
                }
                Text {
                    Layout.fillWidth: true
                    text: "SCALE ISSUE — Scale problem or cartridge taken away. Operator intervention was required. Check scale and loadcell before next cycle."
                    color: "#f5a394"
                    font.pixelSize: 14
                    elide: Text.ElideRight
                }
                MotionButton {
                    Layout.preferredWidth: 110; Layout.preferredHeight: 30
                    text: "✓  Confirm"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#3a1210"; border.color: "#f0735c"; border.width: 1; radius: 4 }
                    contentItem: Text {
                        text: parent.text; color: "#f0735c"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: mainWindow.scaleIssueWarning = false
                }
            }
        }

        // ── Main Content ───────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10

            // Camera Area — 2 rows stacked, 16:9
            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: 770
                Layout.maximumWidth: 770
                spacing: 8

                Rectangle {
                    color: cPanel
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    border.color: cBorder
                    radius: 8
                    clip: true
                    HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }

                    // specular top edge
                    Rectangle {
                        anchors { top: parent.top; left: parent.left; right: parent.right }
                        height: 1; z: 1
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: "transparent" }
                            GradientStop { position: 0.4; color: "#33ffffff" }
                            GradientStop { position: 0.6; color: "#33ffffff" }
                            GradientStop { position: 1.0; color: "transparent" }
                        }
                    }

                    Column {
                        anchors.fill: parent; anchors.margins: 10
                        spacing: 10

                        Repeater {
                            model: camNode.cameraList
                            delegate: CameraView {
                                cameraName: modelData.name
                                topic: modelData.topic
                                providerId: modelData.providerId
                                width: parent.width
                            }
                        }
                    }
                }

            }

            // ── Analog / Pressure Column (Middle) ──────────────────
            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: 400
                Layout.maximumWidth: 400
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8
                HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }

                // specular top edge
                Rectangle {
                    anchors { top: parent.top; left: parent.left; right: parent.right }
                    height: 1; z: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.4; color: "#44ffffff" }
                        GradientStop { position: 0.6; color: "#44ffffff" }
                        GradientStop { position: 1.0; color: "transparent" }
                    }
                }

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 10

                    Text {
                        text: "ANALOG PRESSURE"
                        color: "#ffffff"
                        font.pixelSize: 20
                        font.bold: true
                        font.letterSpacing: 0.6
                        Layout.fillWidth: true
                    }

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        PCard { lbl: "S1 Chamber";   val: hpController.pressureS1; maxVal: 1200; cardIndex: 0 }
                        PCard { lbl: "S2 Cartridge"; val: hpController.pressureS2; maxVal: 1200; cardIndex: 1 }
                        PCard { lbl: "S3 Tank";      val: hpController.pressureS3; maxVal: 1200; cardIndex: 2 }
                    }

                    Item { height: 10 } // Spacer

                    RowLayout {
                        Layout.fillWidth: true
                        Text {
                            text: "CARTRIDGE\nPRESSURE"
                            color: "#74899f"
                            font.pixelSize: 18
                            font.bold: true
                            Layout.fillWidth: true
                        }
                        Text {
                            text: getCartridgeStats()
                            color: "#74899f"
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
                                cartIndex: index
                                cartName: "Cart " + (index + 1)
                                cartVal:  (hpController.cartridgePressures && hpController.cartridgePressures.length > index) ? (Number(hpController.cartridgePressures[index]) || 0) : 0
                            }
                        }
                    }
                }
            }

            // ── System Monitor + Controls ──────────────────────
            Rectangle {
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.minimumWidth: 300
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8
                HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }

                // specular top edge
                Rectangle {
                    anchors { top: parent.top; left: parent.left; right: parent.right }
                    height: 1; z: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.4; color: "#44ffffff" }
                        GradientStop { position: 0.6; color: "#44ffffff" }
                        GradientStop { position: 1.0; color: "transparent" }
                    }
                }

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 6

                    Text {
                        text: "SYSTEM MONITOR"
                        color: "#ffffff"; font.bold: true; font.pixelSize: 20; font.letterSpacing: 0.6
                        Layout.alignment: Qt.AlignHCenter
                    }
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#163a52" }

                    // ROI hong -> vision decision bi khoa (khong chon row/slot).
                    // Truoc day chi co RCLCPP_ERROR trong log, man hinh khong bao gi.
                    Rectangle {
                        Layout.fillWidth: true
                        visible: robotController.roiError !== ""
                        Layout.preferredHeight: visible ? roiErrText.implicitHeight + 12 : 0
                        radius: 6
                        color: Qt.rgba(0.55, 0.10, 0.10, 0.85)
                        border.color: "#ff6b6b"; border.width: 1
                        Text {
                            id: roiErrText
                            anchors.fill: parent; anchors.margins: 6
                            text: "⚠ " + robotController.roiError
                            color: "#ffe0e0"; font.pixelSize: 14; font.bold: true
                            wrapMode: Text.WordWrap
                        }
                    }

                    // ── 3-Column Layout: Left (Robot Status), Middle (Ink Info), Right (Control Mode) ──
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: false
                        spacing: 20

                        // COLUMN 1: Robot Status
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            columns: 2; rowSpacing: 8; columnSpacing: 10

                            Text { text: "Mode:"; color: "#9fb3c8"; font.pixelSize: 18 }
                            Text { text: cameraPageRoot.ctrlMode.toUpperCase(); color: "#d6f1ff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Uptime:"; color: "#9fb3c8"; font.pixelSize: 18 }
                            Text { text: robotController.systemUptime; color: "#d6f1ff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "State Robot:"; color: "#9fb3c8"; font.pixelSize: 18 }
                            Text { text: robotController.systemStatus; color: "#d6f1ff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }
                        }

                        Rectangle { width: 1; Layout.fillHeight: true; color: "#163a52" }

                        // COLUMN 2: Ink & Scale Info
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            columns: 2; rowSpacing: 8; columnSpacing: 10

                            Text { text: "Ink Name:"; color: "#9fb3c8"; font.pixelSize: 18 }
                            Text { text: scaleController.activeInkName; color: "#d6f1ff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Cartridge Type:"; color: "#9fb3c8"; font.pixelSize: 18 }
                            Text { text: scaleController.activeCartName; color: "#d6f1ff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Weight Batch:"; color: "#9fb3c8"; font.pixelSize: 18 }
                            Text { text: scaleController.totalBatchWeight > 0 ? scaleController.totalBatchWeight.toFixed(2) + " g" : "0.00 g"; color: "#d6f1ff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }
                        }

                        Rectangle { width: 1; Layout.fillHeight: true; color: "#163a52" }

                        // COLUMN 3: Control Mode
                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            spacing: 6

                            Text { text: "CONTROL MODE "; color: "#ffffff"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 84
                                radius: 22
                                antialiasing: true
                                color: "#081627"
                                border.color: "#163a52"
                                border.width: 1

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 4
                                    spacing: 0

                                    Repeater {
                                        model: [
                                            { key: "auto",      lbl: "AUTO" },
                                            { key: "camera_ai", lbl: "CAMERA AI" }
                                        ]
                                        delegate: MotionButton {
                                            id: modeOption
                                            required property var modelData
                                            property bool isSelected: cameraPageRoot.ctrlMode === modelData.key
                                            property bool isLocked: cameraPageRoot.modeLocked || cameraPageRoot.robotBusy
                                            Layout.fillWidth: true
                                            Layout.fillHeight: true
                                            hoverScale: 1.02
                                            pressScale: 0.97
                                            shadowColor: "#66000000"
                                            enabled: !isLocked
                                            opacity: (!isSelected && isLocked) ? 0.35 : 1.0
                                            onClicked: {
                                                cameraPageRoot.ctrlMode = modelData.key
                                                mainWindow.syncOperationMode(modelData.key)
                                            }

                                            background: Rectangle {
                                                radius: height / 2
                                                antialiasing: true
                                                color: modeOption.isSelected ? "transparent" : (modeOption.hovered ? Qt.rgba(0.15, 0.55, 0.70, 0.12) : "transparent")
                                                border.color: modeOption.isSelected ? cServoJogBorder : (modeOption.hovered ? cBtnBaseBorder : "transparent")
                                                border.width: modeOption.isSelected || (modeOption.hovered && !modeOption.isSelected) ? 1 : 0
                                                gradient: modeOption.isSelected ? selectedModeGradient : null
                                                Behavior on color { ColorAnimation { duration: 140 } }
                                                Behavior on border.color { ColorAnimation { duration: 140 } }

                                                Gradient {
                                                    id: selectedModeGradient
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: cServoJogStart }
                                                    GradientStop { position: 1.0; color: cServoJogEnd }
                                                }
                                            }

                                            contentItem: Text {
                                                text: (modeOption.isLocked && !modeOption.isSelected ? "🔒 " : "") + modeOption.modelData.lbl
                                                color: modeOption.isSelected ? cServoJogText : "#d6f1ff"
                                                font.pixelSize: 16
                                                font.bold: true
                                                horizontalAlignment: Text.AlignHCenter
                                                verticalAlignment: Text.AlignVCenter
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#163a52" }

                    // Error indicator
                    Rectangle {
                        Layout.fillWidth: true; Layout.fillHeight: false
                        height: (robotController.errorLog || "").length > 0 ? 24 : 0; radius: 4
                        visible: (robotController.errorLog || "").length > 0
                        color: "#160a09"; border.color: "#f0735c"
                        Row { anchors.fill: parent; anchors.leftMargin: 6; spacing: 4
                            Text { text: "⚠"; color: "#f0735c"; font.pixelSize: 17; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: robotController.errorLog; color: "#ecc45a"; font.pixelSize: 17; font.family: "monospace"; anchors.verticalCenter: parent.verticalCenter; elide: Text.ElideRight; width: parent.width - 30 }
                        }
                    }

                    // Input Row
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: false
                        Text { text: "INPUT ROW"; color: "#ffffff"; font.pixelSize: 17; font.bold: true }
                        Text {
                            Layout.fillWidth: true;
                            font.pixelSize: 13; color: "#74899f"; font.italic: true
                            text: cameraPageRoot.rowLocked ? "🔒 Waiting for tray..." : (cameraPageRoot.ctrlMode === "camera_ai" ? "(AI auto)" : "Select then press PICK_CARTRIDGE")
                            elide: Text.ElideRight
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 4
                        Repeater { model: 5
                            delegate: Rectangle {
                                property int rn: index + 1
                                property bool aiMode: cameraPageRoot.ctrlMode === "camera_ai"
                                property bool isActive: robotController.selectedRow === rn
                                property bool isAiDetected: aiMode && (robotController.rowReady[index] === true)
                                property bool canSelect: !cameraPageRoot.rowLocked && cameraPageRoot.ctrlMode === "auto"
                                readonly property bool isAccent: aiMode ? isAiDetected : isActive
                                Layout.fillWidth: true; height: 32; radius: 12
                                color: isAccent ? "transparent"
                                                : (aiMode
                                                   ? Qt.rgba(0.03, 0.11, 0.18, 0.50)
                                                   : Qt.rgba(0.03, 0.11, 0.18, 0.68))
                                border.color: isAccent ? cServoJogBorder : cBtnBaseBorder
                                border.width: isAccent ? 2 : 1
                                gradient: isAccent ? accentGradient : null
                                opacity: aiMode
                                         ? 1.0
                                         : (canSelect ? 1.0 : (isActive ? 1.0 : 0.45))
                                Gradient {
                                    id: accentGradient
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: maRow.pressed ? Qt.darker(cServoJogStart, 1.15) : cServoJogStart }
                                    GradientStop { position: 1.0; color: maRow.pressed ? Qt.darker(cServoJogEnd, 1.15) : cServoJogEnd }
                                }
                                Text {
                                    anchors.centerIn: parent; text: "R" + rn
                                    color: isAccent ? cServoJogText : "#9fb3c8"
                                    font.pixelSize: 18; font.bold: isAccent
                                }
                                MotionMouseArea { id: maRow; anchors.fill: parent; enabled: canSelect; onClicked: robotController.selectRow(rn) }
                            }
                        }
                    }

                    // Output Slot
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: false
                        Text { text: "OUTPUT SLOT"; color: "#ffffff"; font.pixelSize: 17; font.bold: true }
                        Text {
                            text: robotController.selectedSlot > 0 ? ("Selected slot " + robotController.selectedSlot) : "Select output tray position"
                            color: "#74899f"; font.pixelSize: 17
                        }
                    }
                    GridLayout {
                        Layout.fillWidth: true
                        columns: 5
                        columnSpacing: 3
                        rowSpacing: 4
                        Repeater { model: 10
                            delegate: Rectangle {
                                property int sn: index + 1
                                property bool aiMode: cameraPageRoot.ctrlMode === "camera_ai"
                                property bool isReady: aiMode && (robotController.slotReady[index] === true)
                                property bool isActive: robotController.selectedSlot === sn
                                property bool canSelect: cameraPageRoot.ctrlMode === "auto"
                                readonly property bool isAccent: aiMode ? isReady : isActive
                                Layout.fillWidth: true
                                Layout.preferredHeight: 32
                                radius: 12
                                color: isAccent ? "transparent"
                                                : (aiMode
                                                   ? Qt.rgba(0.03, 0.11, 0.18, 0.68)
                                                   : Qt.rgba(0.03, 0.11, 0.18, 0.68))
                                border.color: isAccent ? cServoJogBorder : cBtnBaseBorder
                                border.width: isAccent ? 2 : 1
                                gradient: isAccent ? accentGradient : null
                                opacity: aiMode
                                         ? (isReady || isActive ? 1.0 : 0.45)
                                         : (canSelect ? 1.0 : (isActive ? 1.0 : 0.45))
                                Gradient {
                                    id: accentGradient
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: maSlot.pressed ? Qt.darker(cServoJogStart, 1.15) : cServoJogStart }
                                    GradientStop { position: 1.0; color: maSlot.pressed ? Qt.darker(cServoJogEnd, 1.15) : cServoJogEnd }
                                }
                                Text {
                                    anchors.centerIn: parent; text: "O" + sn
                                    color: isAccent ? cServoJogText : "#9fb3c8"
                                    font.pixelSize: 18; font.bold: isAccent
                                }
                                MotionMouseArea {
                                    id: maSlot
                                    anchors.fill: parent
                                    enabled: canSelect
                                    hoverScale: 1.02
                                    pressScale: 0.98
                                    shadowColor: "#66000000"
                                    pressedShadowColor: "#80000000"
                                    shimmerColor: "#40ffffff"
                                    onClicked: robotController.selectSlot(sn)
                                }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#163a52" }

                    Text { text: "STATE COMMANDS"; color: "#ffffff"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }
                    GridLayout {
                        Layout.fillWidth: true; columns: 2; rowSpacing: 8; columnSpacing: 8
                        Repeater {
                            model: [
                                { lbl: "IN_READY",     displayLbl: "IN_READY",       icon: "icons/between_horizontal_end.svg",   bgStart: cBtnBaseStart,    bgEnd: cBtnBaseEnd,    bc: cBtnBaseBorder,    tc: cBtnBaseText },
                                { lbl: "OUT_READY",    displayLbl: "OUT_READY",      icon: "icons/between_horizontal_start.svg", bgStart: cBtnBaseStart,    bgEnd: cBtnBaseEnd,    bc: cBtnBaseBorder,    tc: cBtnBaseText },
                                { lbl: "PICK_INPUT",   displayLbl: "PICK_CARTRIDGE", icon: "icons/arrows_up_from_line.svg",      bgStart: cServoJogStart,   bgEnd: cServoJogEnd,   bc: cServoJogBorder,   tc: cServoJogText },
                                { lbl: "PICK_CHAMBER", displayLbl: "PICK_CHAMBER",   icon: "icons/fold_horizontal.svg",         bgStart: cServoJogStart,   bgEnd: cServoJogEnd,   bc: cServoJogBorder,   tc: cServoJogText },
                                { lbl: "PLACE_OUTPUT", displayLbl: "PLACE_OUTPUT",   icon: "icons/package.svg",                 bgStart: "#1C4D8D",        bgEnd: "#0c1726",        bc: "#0c1726",        tc: "#ffffff" },
                                { lbl: "PLACE_FAIL",   displayLbl: "PLACE_FAIL",     icon: "icons/package_x.svg",               bgStart: "#E68457",        bgEnd: "#8a4210",        bc: "#8a4210",        tc: "#ffffff" }
                            ]
                            delegate: Rectangle {
                                required property var modelData
                                property bool isActive: (modelData.lbl === "IN_READY" && robotController.inReady) || (modelData.lbl === "OUT_READY" && robotController.outReady)
                                readonly property bool isReadyBtn: modelData.lbl === "IN_READY" || modelData.lbl === "OUT_READY"
                                property color gStart: (isReadyBtn && isActive) ? cBtnClearStart : modelData.bgStart
                                property color gEnd:   (isReadyBtn && isActive) ? cBtnClearEnd : modelData.bgEnd
                                property color labelColor: (isReadyBtn && isActive) ? cBtnClearText : modelData.tc
                                property color roleBorder: (isReadyBtn && isActive) ? cBtnActionBorder : modelData.bc
                                Layout.fillWidth: true; height: 64; radius: 10
                                color: "transparent"
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: ma.pressed ? Qt.darker(gStart, 1.2) : gStart }
                                    GradientStop { position: 1.0; color: ma.pressed ? Qt.darker(gEnd, 1.2) : gEnd }
                                }
                                border.color: ma.containsMouse || ma.pressed ? Qt.lighter(roleBorder, 1.06) : roleBorder
                                border.width: 1
                                Behavior on border.color { ColorAnimation { duration: 110 } }
                                Item {
                                    anchors.fill: parent

                                    Item {
                                        id: stateIconHost
                                        width: 38; height: 36
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        anchors.top: parent.top
                                        anchors.topMargin: 1

                                        Image {
                                            source: modelData.icon.indexOf(".svg") !== -1 ? modelData.icon : ""
                                            visible: source.toString() !== ""
                                            width: 34; height: 34
                                            sourceSize.width: 80
                                            sourceSize.height: 80
                                            fillMode: Image.PreserveAspectFit
                                            smooth: true
                                            mipmap: true
                                            antialiasing: true
                                            anchors.centerIn: parent
                                        }

                                        Text {
                                            text: modelData.icon.indexOf(".svg") === -1 ? modelData.icon : ""
                                            visible: text !== ""
                                            color: parent.parent.parent.labelColor
                                            font.pixelSize: 22
                                            anchors.centerIn: parent
                                        }

                                    }

                                    Row {
                                        spacing: 6
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        anchors.top: stateIconHost.bottom
                                        anchors.bottom: parent.bottom

                                        Rectangle {
                                            width: 9; height: 9; radius: 4.5
                                            color: parent.parent.parent.isActive ? "#67d0ff" : "#14263c"
                                            border.color: cBtnActionBorder
                                            border.width: 1
                                            visible: modelData.lbl === "IN_READY" || modelData.lbl === "OUT_READY"
                                            anchors.verticalCenter: parent.verticalCenter
                                            Behavior on color { ColorAnimation { duration: 200 } }
                                        }

                                        Text {
                                            text: modelData.displayLbl
                                            color: parent.parent.parent.labelColor
                                            font.pixelSize: 13
                                            font.bold: true
                                            anchors.verticalCenter: parent.verticalCenter
                                        }
                                    }
                                }
                                MotionMouseArea {
                                    id: ma
                                    anchors.fill: parent
                                    hoverScale: 1.012
                                    pressScale: 0.99
                                    shadowEnabled: false
                                    shimmerEnabled: false
                                    raiseOnHover: true
                                    onClicked: {
                                    if (modelData.lbl === "IN_READY") robotController.simulateInputTrayReady()
                                    else if (modelData.lbl === "OUT_READY") robotController.simulateOutputTrayReady()
                                    else if (modelData.lbl === "PICK_INPUT") robotController.simulateFeedChamber()
                                    else if (modelData.lbl === "PICK_CHAMBER") robotController.simulateFillDone()
                                    else if (modelData.lbl === "PLACE_OUTPUT") robotController.gotoState("PLACE_TO_OUTPUT")
                                    else if (modelData.lbl === "PLACE_FAIL") robotController.gotoState("PLACE_TO_FAIL")
                                    else robotController.gotoState(modelData.lbl)
                                    }
                                }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#163a52" }

                    Text { text: "SYSTEM CONTROL"; color: "#ffffff"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }

                    GridLayout {
                        Layout.fillWidth: true; columns: 3; rowSpacing: 8; columnSpacing: 8

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 10; color: "transparent"; border.color: startMA.containsMouse || startMA.pressed ? Qt.lighter(cBtnPrimaryEnd, 1.06) : cBtnPrimaryEnd; border.width: 1
                            Behavior on border.color { ColorAnimation { duration: 110 } }
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: startMA.pressed ? Qt.darker(cBtnPrimaryStart, 1.15) : cBtnPrimaryStart }
                                GradientStop { position: 1.0; color: startMA.pressed ? Qt.darker(cBtnPrimaryEnd, 1.15) : cBtnPrimaryEnd }
                            }
                            Row {
                                anchors.centerIn: parent
                                spacing: 8
                                Image {
                                    source: "icons/play.svg"
                                    width: 28
                                    height: 28
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Text {
                                    text: "START"
                                    color: "#d6f1ff"
                                    font.pixelSize: 19
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                            }
                            MotionMouseArea {
                                id: startMA
                                anchors.fill: parent
                                hoverScale: 1.012
                                pressScale: 0.99
                                shadowEnabled: false
                                shimmerEnabled: false
                                raiseOnHover: true
                                onClicked: {
                                if (cameraPageRoot.startCommandLocked)
                                    return
                                cameraPageRoot.startCommandLocked = true
                                cameraPageRoot.pendingStartUiMode = cameraPageRoot.ctrlMode
                                cameraPageRoot.pendingStartMode = mainWindow.cartridgeModeFor(cameraPageRoot.ctrlMode)

                                mainWindow.syncOperationMode(cameraPageRoot.ctrlMode)
                                cameraPageRoot.dispatchStartAfterModeConfirmed()
                                }
                            }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 10; color: "transparent"; border.color: enMA.containsMouse || enMA.pressed ? Qt.lighter("#1A312C", 1.06) : "#1A312C"; border.width: 1
                            Behavior on border.color { ColorAnimation { duration: 110 } }
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: enMA.pressed ? Qt.darker("#3B6978", 1.15) : "#3B6978" }
                                GradientStop { position: 1.0; color: enMA.pressed ? Qt.darker("#1A312C", 1.15) : "#1A312C" }
                            }
                            Row {
                                anchors.centerIn: parent
                                spacing: 8
                                Image {
                                    source: "icons/power_lucide.svg"
                                    width: 27
                                    height: 27
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Text {
                                    text: "ENABLE"
                                    color: "#d6f1ff"
                                    font.pixelSize: 19
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                            }
                            MotionMouseArea {
                                id: enMA
                                anchors.fill: parent
                                hoverScale: 1.012
                                pressScale: 0.99
                                shadowEnabled: false
                                shimmerEnabled: false
                                raiseOnHover: true
                                onClicked: robotController.enableSystem(true)
                            }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 10; color: "transparent"; border.color: stopResetMA.containsMouse || stopResetMA.pressed ? Qt.lighter(cBtnDangerEnd, 1.06) : cBtnDangerEnd; border.width: 1
                            Behavior on border.color { ColorAnimation { duration: 110 } }
                            gradient: Gradient {
                                orientation: Gradient.Vertical
                                GradientStop { position: 0.0; color: stopResetMA.pressed ? Qt.darker("#E05454", 1.25) : "#E05454" }
                                GradientStop { position: 0.5; color: stopResetMA.pressed ? Qt.darker(cBtnEmergencyStart, 1.25) : cBtnEmergencyStart }
                                GradientStop { position: 1.0; color: stopResetMA.pressed ? Qt.darker("#7a2424", 1.25) : "#7a2424" }
                            }
                            Row {
                                anchors.centerIn: parent
                                spacing: 8
                                Image {
                                    source: "icons/octagon_x_lucide.svg"
                                    width: 27
                                    height: 27
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Text {
                                    text: "STOP"
                                    color: "#ffffff"
                                    font.pixelSize: 19
                                    font.bold: true
                                    font.letterSpacing: 1.2
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                            }
                            MotionMouseArea {
                                id: stopResetMA
                                anchors.fill: parent
                                hoverScale: 1.012
                                pressScale: 0.99
                                shadowEnabled: false
                                shimmerEnabled: false
                                raiseOnHover: true
                                onClicked: {
                                cameraPageRoot.modeLocked = false
                                mainWindow.stopSynchronizedSystems()
                                }
                            }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 10; color: "transparent"; border.color: clrMA.containsMouse || clrMA.pressed ? Qt.lighter(cBtnClearEnd, 1.06) : cBtnClearEnd; border.width: 1
                            Behavior on border.color { ColorAnimation { duration: 110 } }
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: clrMA.pressed ? Qt.darker(cBtnClearStart, 1.15) : cBtnClearStart }
                                GradientStop { position: 1.0; color: clrMA.pressed ? Qt.darker(cBtnClearEnd, 1.15) : cBtnClearEnd }
                            }
                            Row {
                                anchors.centerIn: parent
                                spacing: 8
                                Image {
                                    source: "icons/brush_cleaning_white.svg"
                                    width: 27
                                    height: 27
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Text {
                                    text: "CLEAR"
                                    color: "#ffffff"
                                    font.pixelSize: 19
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                            }
                            MotionMouseArea {
                                id: clrMA
                                anchors.fill: parent
                                hoverScale: 1.012
                                pressScale: 0.99
                                shadowEnabled: false
                                shimmerEnabled: false
                                raiseOnHover: true
                                onClicked: robotController.clearError()
                            }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 10; color: "transparent"; border.color: resMA.containsMouse || resMA.pressed ? Qt.lighter(cServoRunEnd, 1.06) : cServoRunEnd; border.width: 1
                            Behavior on border.color { ColorAnimation { duration: 110 } }
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: resMA.pressed ? Qt.darker(cServoRunStart, 1.15) : cServoRunStart }
                                GradientStop { position: 1.0; color: resMA.pressed ? Qt.darker(cServoRunEnd, 1.15) : cServoRunEnd }
                            }
                            Row {
                                anchors.centerIn: parent
                                spacing: 8
                                Image {
                                    source: "icons/step_forward.svg"
                                    width: 28
                                    height: 28
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Text {
                                    text: "RESUME"
                                    color: cServoRunText
                                    font.pixelSize: 19
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                            }
                            MotionMouseArea {
                                id: resMA
                                anchors.fill: parent
                                hoverScale: 1.012
                                pressScale: 0.99
                                shadowEnabled: false
                                shimmerEnabled: false
                                raiseOnHover: true
                                onClicked: { cameraPageRoot.pauseLatched = false; robotController.resumeRobot() }
                            }
                        }

                        Rectangle {
                            Layout.fillWidth: true; height: 52; radius: 10; color: "transparent"
                            border.color: pauseMA.containsMouse || pauseMA.pressed || cameraPageRoot.pauseLatched ? Qt.lighter("#8a4210", 1.06) : "#8a4210"
                            border.width: cameraPageRoot.pauseLatched ? 2 : 1
                            transform: Translate { y: cameraPageRoot.pauseLatched ? 2 : 0 }
                            Behavior on border.color { ColorAnimation { duration: 110 } }
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop {
                                    position: 0.0
                                    color: pauseMA.pressed || cameraPageRoot.pauseLatched
                                           ? Qt.darker("#8a4210", 1.12)
                                           : "#8a4210"
                                }
                                GradientStop {
                                    position: 1.0
                                    color: pauseMA.pressed || cameraPageRoot.pauseLatched
                                           ? Qt.darker("#E68457", 1.12)
                                           : "#E68457"
                                }
                            }
                            Rectangle {
                                anchors.fill: parent
                                radius: parent.radius
                                color: "#000000"
                                opacity: cameraPageRoot.pauseLatched ? 0.22 : 0.0
                            }
                            Row {
                                anchors.centerIn: parent
                                spacing: 8
                                Image {
                                    source: "icons/circle_pause_lucide.svg"
                                    width: 27
                                    height: 27
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Text {
                                    text: "PAUSE"
                                    color: "#ffffff"
                                    font.pixelSize: 19
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                            }
                            MotionMouseArea {
                                id: pauseMA
                                anchors.fill: parent
                                hoverScale: 1.012
                                pressScale: 0.99
                                shadowEnabled: false
                                shimmerEnabled: false
                                raiseOnHover: true
                                onClicked: { cameraPageRoot.pauseLatched = true; robotController.pauseRobot() }
                            }
                        }
                    }

                }
            }
        }

        // ── Footer ─────────────────────────────────────────────
        Item {
            height: 40; Layout.fillWidth: true
            Rectangle {
                anchors.fill: parent; color: cPanel; border.color: cBorder; border.width: 1
                RowLayout {
                    anchors.fill: parent; anchors.margins: 10
                    Text { text: "© 2025 RYNAN TECHNOLOGIES"; color: "#ffffff"; font.pixelSize: 16; Layout.alignment: Qt.AlignVCenter }
                    Item { Layout.fillWidth: true }
                    RowLayout { spacing: 6
                        Image { source: "qrc:/icons/qml/icons/app_badging.svg"; width: 24; height: 24; fillMode: Image.PreserveAspectFit; smooth: true; Layout.preferredWidth: 24; Layout.preferredHeight: 24; Layout.alignment: Qt.AlignVCenter }
                        Text { text: "Status: Running"; color: "#3ed0b4"; font.pixelSize: 16; Layout.alignment: Qt.AlignVCenter }
                    }
                    Rectangle { width: 2; Layout.fillHeight: true; color: "#163a52" }
                    RowLayout { spacing: 6; Layout.alignment: Qt.AlignVCenter
                        Image { source: "qrc:/icons/qml/icons/schedule.svg"; fillMode: Image.PreserveAspectFit; smooth: true; Layout.preferredWidth: 24; Layout.preferredHeight: 24; Layout.alignment: Qt.AlignVCenter }
                        Text { text: currentTime; font.pixelSize: 16; color: "#ffffff"; Layout.alignment: Qt.AlignVCenter }
                    }
                }
            }
        }
    }

    component PCard: Rectangle {
        property string lbl: ""
        property real   val: 0
        property real   maxVal: 1000
        property int    cardIndex: 0
        readonly property string cls: classifyPressure(val, maxVal * 0.18, maxVal * 0.35, maxVal * 0.90)
        Layout.fillWidth: true
        Layout.fillHeight: true
        implicitHeight: 64
        radius: 10
        color: cPanel2
        clip: true
        border.color: "#163a52"
        border.width: 1
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? "#7fcdf5" : "#163a52" }
        ColumnLayout {
            anchors.fill: parent
            anchors.leftMargin: 16
            anchors.rightMargin: 16
            anchors.topMargin: 10
            anchors.bottomMargin: 10
            spacing: 8
            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 8
                Text {
                    text: {
                        var parts = lbl.split(" ");
                        var head = parts[0];
                        var tail = parts.length > 1 ? parts.slice(1).join(" ") : "";
                        return tail.length > 0 ? (head + " · " + tail) : head;
                    }
                    color: cBtnBaseText
                    font.pixelSize: 15
                    font.bold: true
                    Layout.alignment: Qt.AlignVCenter
                }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 4
                    Layout.alignment: Qt.AlignVCenter
                    Text {
                        text: val.toFixed(1)
                        color: pressureTextColor(cls)
                        font.pixelSize: 26
                        font.bold: true
                        font.family: "monospace"
                    }
                    Text {
                        text: "mbar"
                        color: pressureTextColor(cls)
                        font.pixelSize: 13
                        font.bold: true
                        Layout.alignment: Qt.AlignBottom
                        Layout.bottomMargin: 3
                    }
                }
            }
            Rectangle {
                id: pcBarContainer
                Layout.fillWidth: true
                Layout.preferredHeight: 12
                radius: height / 2
                antialiasing: true
                color: pressureTrackColor(cls)
                border.color: "#22445c"
                border.width: 1
                clip: true

                Rectangle {
                    id: pcFilledBar
                    height: parent.height
                    radius: height / 2
                    antialiasing: true
                    width: parent.width * Math.max(0, Math.min(1, val / maxVal))
                    clip: true
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: pressureFillStart(cls) }
                        GradientStop { position: 0.55; color: "#1e9ed0" }
                        GradientStop { position: 1.0; color: pressureFillEnd(cls) }
                    }

                    onWidthChanged: {
                        pcShimmerAnim.restart()
                    }

                    Rectangle {
                        id: pcBarShimmer
                        height: parent.height
                        width: 160
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0;  color: "#00ffffff" }
                            GradientStop { position: 0.35; color: "#22ffffff" }
                            GradientStop { position: 0.5;  color: "#44ffffff" }
                            GradientStop { position: 0.65; color: "#22ffffff" }
                            GradientStop { position: 1.0;  color: "#00ffffff" }
                        }
                        PropertyAnimation {
                            id: pcShimmerAnim
                            target: pcBarShimmer
                            property: "x"
                            from: -pcBarShimmer.width
                            to: pcFilledBar.width
                            duration: 2600 + cardIndex * 300
                            loops: Animation.Infinite
                            running: pcFilledBar.width > 0
                        }
                    }
                }
            }
        }
    }

    component CartRow: Rectangle {
        property string cartName: ""
        property real   cartVal: 0
        property int    cartIndex: 0
        readonly property string cls: classifyPressure(cartVal, 280, 400, 600)
        Layout.fillWidth: true
        Layout.fillHeight: true
        implicitHeight: 60
        radius: 6
        color:        cPanel2
        clip:         true
        border.color: "#163a52"
        border.width: 1
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? "#7fcdf5" : "#163a52" }
        ColumnLayout {
            anchors.fill: parent
            anchors.leftMargin: 14
            anchors.rightMargin: 14
            anchors.topMargin: 8
            anchors.bottomMargin: 8
            spacing: 2
            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                ColumnLayout {
                    spacing: 0
                    Layout.alignment: Qt.AlignVCenter
                    Text {
                        text: {
                            var parts = cartName.split(" ");
                            return parts[0].toUpperCase();
                        }
                        color: cBtnBaseText
                        font.pixelSize: 11
                        font.bold: true
                    }
                    Text {
                        text: {
                            var parts = cartName.split(" ");
                            return parts.length > 1 ? parts[1] : "";
                        }
                        color: cBtnBaseText
                        font.pixelSize: 16
                        font.bold: true
                    }
                }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 4
                    Layout.alignment: Qt.AlignVCenter
                    Text {
                        text: cartVal.toFixed(1)
                        color: pressureTextColor(cls)
                        font.pixelSize: 21
                        font.bold: true
                        font.family: "monospace"
                    }
                    Text {
                        text: "mbar"
                        color: pressureTextColor(cls)
                        font.pixelSize: 13
                        font.bold: true
                        Layout.alignment: Qt.AlignBottom
                        Layout.bottomMargin: 2
                    }
                }
            }
            Rectangle {
                id: barContainer
                Layout.fillWidth: true
                Layout.preferredHeight: 12
                radius: height / 2
                antialiasing: true
                color: pressureTrackColor(cls)
                border.color: "#22445c"
                border.width: 1
                clip: true

                Rectangle {
                    id: cartFilledBar
                    height: parent.height
                    radius: height / 2
                    antialiasing: true
                    width: parent.width * Math.max(0, Math.min(1, cartVal / 1200))
                    clip: true
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: pressureFillStart(cls) }
                        GradientStop { position: 0.55; color: "#1e9ed0" }
                        GradientStop { position: 1.0; color: pressureFillEnd(cls) }
                    }

                    onWidthChanged: {
                        cartShimmerAnim.restart()
                    }

                    Rectangle {
                        id: cartBarShimmer
                        height: parent.height
                        width: 160
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0;  color: "#00ffffff" }
                            GradientStop { position: 0.35; color: "#22ffffff" }
                            GradientStop { position: 0.5;  color: "#44ffffff" }
                            GradientStop { position: 0.65; color: "#22ffffff" }
                            GradientStop { position: 1.0;  color: "#00ffffff" }
                        }
                        PropertyAnimation {
                            id: cartShimmerAnim
                            target: cartBarShimmer
                            property: "x"
                            from: -cartBarShimmer.width
                            to: cartFilledBar.width
                            duration: 2600 + cartIndex * 300
                            loops: Animation.Infinite
                            running: cartFilledBar.width > 0
                        }
                    }
                }
            }
        }
    }

}
