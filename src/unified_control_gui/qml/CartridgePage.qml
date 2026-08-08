import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtGraphicalEffects 1.15

    // NOTE: "cartridge systems" refers strictly to this "ROS2 - CARTRIDGE PROVISION SYSTEM" page (CartridgePage.qml).
    // UI styling can change freely, but keep button logic intact unless the request explicitly says to change behavior.

    // ─── CSS VARIABLES (from cartridge_gui.py) ──────────────────────────────────
    // --bg:       #06101d   page background
    // --bg2:      #0c1726   card background
    // --card:     #06101d   servo/sensor item bg
    // --border:   #0c1726
    // --accent:   #6f4be0
    // --green:    #3ed0b4   --red: #f0735c   --orange: #f5a623
    // --cyan:     #36b6ff   --yellow: #ecc45a   --dim: #74899f   --text: #c7dcef
    //
    // GRID: 230px | 1fr | 220px  /  rows: 3fr top + 2.5fr log
    // areas: "ctrl center servo" / "log log servo"
    // gap:4px padding:6px  height:calc(100vh - 56px)
    // ────────────────────────────────────────────────────────────────────────────
    Item {
        id: root
        anchors.fill: parent
        focus: true
        activeFocusOnTab: true

        readonly property int headerH:  70
        readonly property int tabbarH:  64
        readonly property int tabbarDockGap: 4
        readonly property int gap:       8
        readonly property int pad:       10
        readonly property int ctrlW:   245   // rộng hơn để chứa title font 14
        readonly property int sensorW: 250
        readonly property real rowRatio: 5.0 / (5.0 + 1.6)   // top:log = 5:1.6 → log nhỏ hơn nữa

        property int gridH:   height - headerH - tabbarH - tabbarDockGap
        property int outerW:  width  - pad * 2
        property int outerH:  gridH  - pad * 2
        property int centerW: outerW - ctrlW - sensorW - gap * 2
        property int topH:    Math.floor(outerH * rowRatio) - gap
        property int logH:    outerH - topH - gap
        property int previousStackIndex: 0
        property int slideDirection: 1
        property int screenDragStartIndex: 0
        property bool startCommandLocked: false
        property bool suppressJogEchoForManual: false
        property bool pauseLatched: false
        // Even a tiny infinite opacity animation keeps Qt Quick's scene graph
        // rendering at display refresh rate. Static colors still expose every
        // state while avoiding a permanently busy QSGRenderThread.
        property bool continuousAnimationsEnabled: false
        property Item activeDataInput: null
        readonly property bool viewerContentLocked: mainWindow.isViewer
        readonly property bool operatorTechnicalLocked: mainWindow.isOperator && stack.currentIndex === 1
        readonly property bool contentReadOnly: viewerContentLocked || operatorTechnicalLocked
        readonly property string currentUiMode: mainWindow.selectedCartridgeMode !== ""
                                                ? mainWindow.selectedCartridgeMode
                                                : cartridgeController.currentMode

        readonly property color cBg:     "transparent"
        readonly property color cBg2:    "#990d1e32"
        readonly property color cCard:   "#8806101d"
        readonly property color cBorder: "#1affffff"
        readonly property color cAccent: "#7fcdf5"
        readonly property color cGreen:  "#67d0ff"
        readonly property color cRed:    "#f0735c"
        readonly property color cOrange: "#f5a623"
        readonly property color cCyan:   "#36b6ff"
        readonly property color cYellow: "#ecc45a"
        readonly property color cDim:    "#74899f"
        readonly property color cText:   "#c7dcef"
        readonly property color cWhiteText: "#ffffff"
        readonly property color cCardTitle: "#ffffff"
        readonly property color cTabSelectedTop: Qt.rgba(0.88, 0.95, 1.00, 0.62)
        readonly property color cTabSelectedMid: Qt.rgba(0.82, 0.91, 0.99, 0.55)
        readonly property color cTabSelectedBottom: Qt.rgba(0.72, 0.85, 0.96, 0.48)
        readonly property color cTabSelectedBorder: Qt.rgba(1, 1, 1, 0.28)
        readonly property color cTabSelectedContent: "#06101d"
        readonly property color cHover:  "#40ffffff"
        // Shared liquid-glass action palette — aligned with CameraPage.
        readonly property color cBtnBaseStart:      "#0c1726"
        readonly property color cBtnBaseEnd:        "#06101d"
        readonly property color cBtnBaseBorder:     "#163a52"
        readonly property color cBtnBaseText:       "#d6f1ff"
        readonly property color cBtnPrimaryStart:   "#1f9e86"
        readonly property color cBtnPrimaryEnd:     "#163a52"
        readonly property color cBtnPrimaryBorder:  cBtnPrimaryEnd
        readonly property color cBtnActionStart:    "#163a52"
        readonly property color cBtnActionEnd:      "#081627"
        readonly property color cBtnActionBorder:   "#163a52"
        readonly property color cBtnEnableStart:    "#3B6978"
        readonly property color cBtnEnableEnd:      "#1A312C"
        readonly property color cBtnEnableText:     "#d6f1ff"
        readonly property color cCylinderActiveStart:cBtnClearStart
        readonly property color cCylinderActiveEnd:  cBtnClearEnd
        readonly property color cCylinderActiveText: cBtnClearText
        readonly property color cBtnHomingStart:    "#3B6F8E"
        readonly property color cBtnHomingEnd:      "#1B4058"
        readonly property color cBtnHomingBorder:   cBtnHomingEnd
        readonly property color cBtnHomingText:     "#ffffff"
        readonly property color cBtnClearStart:     "#234C6A"
        readonly property color cBtnClearEnd:       "#102739"
        readonly property color cBtnClearText:      "#ffffff"
        readonly property color cBtnWarningStart:   "#8a4210"
        readonly property color cBtnWarningEnd:     "#E68457"
        readonly property color cBtnWarningBorder:  cBtnWarningStart
        readonly property color cBtnDangerStart:    "#E05454"
        readonly property color cBtnDangerEnd:      "#7a2424"
        readonly property color cBtnDangerBorder:   cBtnDangerEnd
        readonly property color cBtnEmergencyStart: "#E05454"
        readonly property color cBtnEmergencyEnd:   "#9c3030"
        readonly property color cBtnEmergencyBorder: cBtnEmergencyEnd
        readonly property color cServoJogStart:     "#1a4a6e"
        readonly property color cServoJogEnd:       "#0c1726"
        readonly property color cServoJogBorder:    cServoJogEnd
        readonly property color cServoJogText:      "#ffffff"
        readonly property color cServoRunStart:     "#1a4a6e"
        readonly property color cServoRunEnd:       "#0c1726"
        readonly property color cServoRunBorder:    cServoRunEnd
        readonly property color cServoRunText:      "#ffffff"
        readonly property color cUnifiedBtn: cBtnBaseStart
        readonly property color cInReadyLayer: cBtnBaseStart
        readonly property color cInReadyLayerEnd: cBtnBaseEnd
        readonly property color cInReadyBorder: cBtnBaseBorder
        readonly property color cProvisionButton: cBtnBaseStart
        readonly property color cProvisionButtonEnd: cBtnBaseEnd
        readonly property color cProvisionButtonBorder: cBtnBaseBorder
        readonly property color cProvisionButtonText: cBtnBaseText
        readonly property color cFunctionLabelText: "#bfe0f5"
        // Ô hiển thị/nhập giá trị: nền kính tối + viền cyan mờ — KHÔNG dùng màu
        // xanh action (#1f86e0) để phân biệt rõ "ô dữ liệu" với "nút bấm".
        readonly property color cFunctionFieldStart: Qt.rgba(0.06, 0.19, 0.26, 0.82)
        readonly property color cFunctionFieldEnd: Qt.rgba(0.04, 0.13, 0.19, 0.82)
        readonly property color cFunctionFieldBorder: "#4d67d0ff"
        readonly property color cFunctionFieldText: "#d6f1ff"
        readonly property real cPressDarken: 1.18
        readonly property real cPressGradientDarken: 1.15
        readonly property real cPressCustomDarken: 1.12
        readonly property color cBlueWhiteBtn: cBtnActionStart
        readonly property color cBlueWhiteSelected: "#0c1726"
        readonly property color cBlueWhiteIdle: cBtnBaseStart
        readonly property color cBlueWhiteBorder: cBtnActionBorder
        readonly property color cBlueWhiteText: "#ffffff"
        readonly property color cBlueWhiteSubText: "#bfe0f5"
        readonly property color cStateAuxBtn: cBtnBaseStart
        readonly property color cStateAuxBtnEnd: cBtnBaseEnd
        readonly property color cStateAuxBorder: cBtnBaseBorder
        readonly property color cStateAuxText: cBtnBaseText
        readonly property color cFieldBorder: cFunctionFieldBorder
        readonly property color cDashPanel: Qt.rgba(0.06, 0.10, 0.16, 0.34)
        readonly property color cDashCard: cDashPanel
        // Đồng bộ navy-teal với CameraPage: panel = cPanel, card = cPanel2,
        // viền trắng mờ (bỏ viền teal sáng) cho cảm giác glass giống CameraPage.
        readonly property color cControlPanel: cBg2
        readonly property color cControlCard: cCard
        readonly property color cControlBorder: cBorder
        readonly property color cDashboardActionStart: cBtnActionStart
        readonly property color cDashboardActionEnd: cBtnActionEnd
        readonly property color cDashboardActionBorder: cBtnActionBorder
        readonly property color cDashboardActionText: "#ffffff"
        readonly property color cModeSelectedTop: "#A1C2BD"
        readonly property color cModeSelectedMid: "#5C8580"
        readonly property color cModeSelectedBottom: "#163a52"
        readonly property color cModeSelectedBorder: "#163a52"
        readonly property color cModeSelectedText: "#06101d"
        readonly property color cDashCardInner: Qt.rgba(cUnifiedBtn.r, cUnifiedBtn.g, cUnifiedBtn.b, 0.82)
        readonly property color cDashCardField: Qt.rgba(cUnifiedBtn.r, cUnifiedBtn.g, cUnifiedBtn.b, 0.82)
        readonly property color cDashCardBorder: cBorder
        readonly property color cDashCardBorderHover: cHover
        readonly property color cDashButton: cProvisionButton
        readonly property color cDashButtonEnd: cProvisionButtonEnd
        readonly property color cDashButtonBorder: cProvisionButtonBorder
        readonly property color cDashButtonText: cProvisionButtonText
        readonly property color cJogNegativeButton: cServoRunStart
        readonly property color cJogNegativeButtonEnd: cServoRunEnd
        readonly property color cJogNegativeButtonBorder: cServoRunBorder
        readonly property color cMovJButton: cStateAuxBtn
        readonly property color cMovJButtonEnd: cStateAuxBtnEnd
        readonly property color cMovJButtonBorder: cStateAuxBorder
        readonly property color cGetButton: cDashboardActionStart
        readonly property color cGetButtonEnd: cDashboardActionEnd
        readonly property color cGetButtonBorder: cDashboardActionBorder
        readonly property color cSensorIdleBg: Qt.rgba(0.03, 0.11, 0.18, 0.18)
        readonly property color cSensorIdleBorder: Qt.rgba(0.08, 0.22, 0.32, 0.42)
        readonly property color cSensorIdleText: Qt.rgba(0.62, 0.70, 0.78, 0.55)
        readonly property color cSensorIdleDot: Qt.rgba(0.08, 0.22, 0.32, 0.34)
        readonly property color cSensorActiveStart: "#CAE8D5"
        readonly property color cSensorActiveEnd: "#163a52"
        readonly property color cSensorActiveBorder: "#163a52"
        readonly property color cSensorActiveText: "#06101d"
        property bool jogStopStateHint: false
        property bool homingCommandLocked: false

        function pressColor(colorValue) {
            return Qt.darker(colorValue, cPressCustomDarken)
        }

        function pressGradientColor(colorValue) {
            return Qt.darker(colorValue, cPressGradientDarken)
        }

        function homingBusy() {
            return homingCommandLocked || cartridgeController.systemState.toLowerCase().indexOf("homing") !== -1
        }

        function cancelHoming() {
            root.homingCommandLocked = false
            homingLockFailsafeTimer.stop()
            cartridgeController.abortToJog()
            cartridgeController.softStop()
        }

        function stopManualMotionOnly() {
            robotController.stopMotionOnly()
            for (var sid = 1; sid <= 5; ++sid)
                cartridgeController.jogStop(sid)
        }

        function cartridgeStateActive() {
            var globalState = (cartridgeController.systemState || "").toLowerCase()
            var inState = (cartridgeController.stateIn || "").toLowerCase()
            var outState = (cartridgeController.stateOut || "").toLowerCase()
            function busyState(s) {
                return s !== "" && s !== "idle" && s !== "unknown"
            }
            return busyState(globalState) || busyState(inState) || busyState(outState)
        }

        function state1Active() {
            var s = (cartridgeController.stateIn || "").toLowerCase()
            return s.indexOf("s1_") === 0 || s.indexOf("state1") !== -1
        }

        function state2Active() {
            var s = (cartridgeController.stateIn || "").toLowerCase()
            return s.indexOf("s2") === 0 || s.indexOf("state2") !== -1
        }

        function state3Active() {
            var s = (cartridgeController.stateOut || "").toLowerCase()
            return s.indexOf("s3_") === 0 || s.indexOf("state3") !== -1
        }

        function state4Active() {
            var s = (cartridgeController.stateOut || "").toLowerCase()
            return s.indexOf("s4_") === 0 || s.indexOf("state4") !== -1
        }

        function abortStateToJog() {
            robotController.stopMotionOnly()
            cartridgeController.abortToJog()
            mainWindow.selectedCartridgeMode = "jog"
            root.jogStopStateHint = false
        }

        function stopFromSystemControl() {
            mainWindow.stopSynchronizedSystems()
        }

        function showJogStopStateHint() {
            jogStopStateHint = true
            jogStopStateHintTimer.restart()
        }

        function fadeJogStopStateHint() {
            if (jogStopStateHint)
                jogStopStateHintTimer.restart()
        }

        Timer {
            id: jogStopStateHintTimer
            interval: 1200
            repeat: false
            onTriggered: root.jogStopStateHint = false
        }

        Timer {
            id: homingLockFailsafeTimer
            interval: 8000
            repeat: false
            onTriggered: {
                if (cartridgeController.systemState.toLowerCase().indexOf("homing") === -1)
                    root.homingCommandLocked = false
            }
        }

        Connections {
            target: cartridgeController
            function onSystemStateChanged() {
                if (cartridgeController.systemState.toLowerCase().indexOf("homing") === -1)
                    root.homingCommandLocked = false
            }
            function onCurrentModeChanged() {
                if (cartridgeController.currentMode !== "")
                    mainWindow.selectedCartridgeMode = cartridgeController.currentMode
                if (cartridgeController.currentMode === "manual") {
                    root.suppressJogEchoForManual = false
                }
            }
        }

        Connections {
            target: mainWindow
            function onSynchronizedStartRequested(mode) {
                root.startCommandLocked = true
            }
            function onSynchronizedStopRequested() {
                root.startCommandLocked = false
                root.homingCommandLocked = false
                root.pauseLatched = false
                homingLockFailsafeTimer.stop()
            }
        }

        function setStackIndex(nextIndex) {
            var clamped = Math.max(0, Math.min(5, nextIndex))
            if (clamped === stack.currentIndex)
                return

            dismissDataInput()
            previousStackIndex = stack.currentIndex
            slideDirection = clamped > previousStackIndex ? 1 : -1
            stack.currentIndex = clamped
            stackSlide.x = slideDirection * Math.min(140, Math.max(70, stack.width * 0.10))
            stack.opacity = 0.68
            stackSlideAnim.restart()
        }

        function registerDataInput(input) {
            activeDataInput = input
        }

        function unregisterDataInput(input) {
            if (activeDataInput === input)
                activeDataInput = null
        }

        function dismissDataInput() {
            if (!activeDataInput)
                return

            activeDataInput.deselect()
            activeDataInput.focus = false
            activeDataInput = null
            forceActiveFocus(Qt.MouseFocusReason)
            Qt.inputMethod.hide()
        }

        function dismissDataInputOutside(x, y) {
            if (!activeDataInput)
                return

            var localPoint = activeDataInput.mapFromItem(root, x, y)
            if (localPoint.x < 0 || localPoint.y < 0
                    || localPoint.x > activeDataInput.width
                    || localPoint.y > activeDataInput.height)
                dismissDataInput()
        }

        Shortcut {
            sequence: "Left"
            context: Qt.WindowShortcut
            enabled: root.visible && stack.currentIndex > 0
            onActivated: root.setStackIndex(stack.currentIndex - 1)
        }

        Shortcut {
            sequence: "Right"
            context: Qt.WindowShortcut
            enabled: root.visible && stack.currentIndex < 5
            onActivated: root.setStackIndex(stack.currentIndex + 1)
        }

        Component.onCompleted: forceActiveFocus()
        onVisibleChanged: {
            if (visible)
                forceActiveFocus()
            else
                dismissDataInput()
        }

        MouseArea {
            anchors.fill: parent
            z: 10000
            enabled: root.activeDataInput !== null
            propagateComposedEvents: true
            onPressed: {
                root.dismissDataInputOutside(mouse.x, mouse.y)
                mouse.accepted = false
            }
        }

        // Ambient glow blobs — creates depth behind glass panels
        Canvas {
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)
                var g1 = ctx.createRadialGradient(width * 0.85, -height * 0.08, 0, width * 0.85, -height * 0.08, height * 0.80)
                g1.addColorStop(0.00, "rgba(30,100,220,0.42)")
                g1.addColorStop(0.30, "rgba(30,100,220,0.18)")
                g1.addColorStop(0.60, "rgba(30,100,220,0.04)")
                g1.addColorStop(1.00, "rgba(30,100,220,0.00)")
                ctx.fillStyle = g1; ctx.fillRect(0, 0, width, height)
                var g2 = ctx.createRadialGradient(width * 0.06, height * 1.08, 0, width * 0.06, height * 1.08, height * 0.70)
                g2.addColorStop(0.00, "rgba(0,180,160,0.32)")
                g2.addColorStop(0.38, "rgba(0,180,160,0.10)")
                g2.addColorStop(0.68, "rgba(0,180,160,0.02)")
                g2.addColorStop(1.00, "rgba(0,180,160,0.00)")
                ctx.fillStyle = g2; ctx.fillRect(0, 0, width, height)
            }
        }

        // ════════════════════════════════════════════════════════════
        // HEADER
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: header
            anchors { top: parent.top; left: parent.left; right: parent.right }
            height: root.headerH
            color: "#cc0d1428"; border.color: root.cBorder; z: 10

            // Top specular line
            Rectangle {
                anchors { top: parent.top; left: parent.left; right: parent.right }
                height: 1
                gradient: Gradient {
                    orientation: Gradient.Horizontal
                    GradientStop { position: 0.0; color: "transparent" }
                    GradientStop { position: 0.4; color: "#55ffffff" }
                    GradientStop { position: 0.6; color: "#55ffffff" }
                    GradientStop { position: 1.0; color: "transparent" }
                }
            }

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                spacing: 8
                MotionButton {
                    id: backBtn
                    Layout.preferredWidth: 50; Layout.preferredHeight: 50
                    hoverScale: 1.012
                    pressScale: 0.99
                    onClicked: stackView.pop()
                    background: Rectangle {
                        radius: 6
                        color: "transparent"
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: backBtn.pressed ? Qt.darker(root.cServoRunStart, 1.15) : root.cServoRunStart }
                            GradientStop { position: 1.0; color: backBtn.pressed ? Qt.darker(root.cServoRunEnd, 1.15) : root.cServoRunEnd }
                        }
                        border.color: root.cServoRunEnd
                        border.width: 1
                    }
                    contentItem: Image {
                        id: backIcon
                        source: "qrc:/icons/qml/icons/reply_arrow.svg"
                        width: 24; height: 24
                        anchors.centerIn: parent
                        fillMode: Image.PreserveAspectFit; smooth: true
                        visible: false
                    }
                    ColorOverlay {
                        anchors.fill: backIcon
                        source: backIcon
                        color: root.cWhiteText
                    }
                    HoverHint {
                        visible: backBtn.hovered
                        label: "Quay lại"
                        bc: root.cServoRunEnd
                        tc: root.cServoRunText
                    }
                }
                ScreenshotButton {
                    Layout.preferredWidth: 50; Layout.preferredHeight: 50
                    hintText: "Chụp ảnh"
                    onCaptureRequested: {
                        robotController.captureScreenshot()
                    }
                }
                MotionButton {
                    id: restartNodesBtn
                    Layout.preferredWidth: 50; Layout.preferredHeight: 50
                    hoverScale: 1.012
                    pressScale: 0.99
                    onClicked: robotController.restartSystemNodes()
                    background: Rectangle {
                        radius: 6
                        color: "transparent"
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: restartNodesBtn.pressed ? Qt.darker(root.cServoRunStart, 1.15) : root.cServoRunStart }
                            GradientStop { position: 1.0; color: restartNodesBtn.pressed ? Qt.darker(root.cServoRunEnd, 1.15) : root.cServoRunEnd }
                        }
                        border.color: root.cServoRunEnd
                        border.width: 1
                    }
                    contentItem: Item {
                        Image {
                            source: "qrc:/qml/icons/list_restart.svg"
                            width: 34; height: 34
                            anchors.centerIn: parent
                            fillMode: Image.PreserveAspectFit; smooth: true
                        }
                        HoverHint {
                            visible: restartNodesBtn.hovered
                            label: "Restart Node"
                            bc: root.cServoRunEnd
                            tc: root.cServoRunText
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
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: restartGuiBtn.pressed ? Qt.darker(root.cServoRunStart, 1.15) : root.cServoRunStart }
                            GradientStop { position: 1.0; color: restartGuiBtn.pressed ? Qt.darker(root.cServoRunEnd, 1.15) : root.cServoRunEnd }
                        }
                        border.color: root.cServoRunEnd
                        border.width: 1
                    }
                    contentItem: Item {
                        Image {
                            source: "qrc:/qml/icons/refresh_cw.svg"
                            width: 34; height: 34
                            anchors.centerIn: parent
                            fillMode: Image.PreserveAspectFit; smooth: true
                        }
                        HoverHint {
                            visible: restartGuiBtn.hovered
                            label: "Restart GUI"
                            bc: root.cServoRunEnd
                            tc: root.cServoRunText
                        }
                    }
                }
                Item { width: 6 }
                Item { Layout.fillWidth: true }
                Rectangle {
                    id: stateBadge
                    Layout.preferredHeight: 50; radius: 6
                    Layout.preferredWidth: sbRow.implicitWidth + 24
                    color: Qt.rgba(0.03, 0.09, 0.16, 0.45); border.color: root.cBorder; border.width: 1
                    Row {
                        id: sbRow
                        anchors.centerIn: parent; spacing: 8
                        Rectangle {
                            id: stateDot
                            width: 9; height: 9; radius: 4.5
                            anchors.verticalCenter: parent.verticalCenter
                            color: {
                                var s = cartridgeController.systemState.toUpperCase()
                                if (s.indexOf("ERROR") !== -1) return root.cRed
                                if (s === "IDLE" || s === "UNKNOWN" || s === "") return root.cOrange
                                return root.cGreen
                            }
                            SequentialAnimation on opacity {
                                loops: Animation.Infinite
                                running: root.continuousAnimationsEnabled
                                NumberAnimation { to: 0.35; duration: 900 }
                                NumberAnimation { to: 1.0;  duration: 900 }
                            }
                        }
                        Text {
                            text: cartridgeController.systemState.toUpperCase().replace(/\|/g, "   •   ")
                            color: root.cWhiteText
                            font.pixelSize: 14; font.bold: true; font.letterSpacing: 1
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                }
                Item { width: 4 }
                Rectangle {
                    id: modePill; Layout.preferredHeight: 50; radius: 6
                    property string m: root.currentUiMode
                    property bool isIdle: m === "idle" || m === ""
                    Layout.preferredWidth: mpRow.implicitWidth + 26
                    color: Qt.rgba(0.03, 0.09, 0.16, 0.45); border.color: root.cBorder; border.width: 1

                    // Nhấp nháy khi chưa chọn mode
                    SequentialAnimation on opacity {
                        loops: Animation.Infinite
                        running: root.continuousAnimationsEnabled && modePill.isIdle
                        NumberAnimation { to: 0.4; duration: 600 }
                        NumberAnimation { to: 1.0; duration: 600 }
                    }
                    opacity: modePill.isIdle ? 1.0 : 1.0

                    Row {
                        id: mpRow
                        anchors.centerIn: parent
                        spacing: modePill.isIdle ? 7 : 0

                        Image {
                            visible: modePill.isIdle
                            source: "qrc:/qml/icons/message_circle_warning.svg"
                            width: 22; height: 22
                            anchors.verticalCenter: parent.verticalCenter
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                        }

                        Text {
                            id: mpLbl
                            text: modePill.isIdle ? "SELECT MODE" : modePill.m.toUpperCase()
                            color: root.cWhiteText
                            font.pixelSize: 14; font.bold: true; font.letterSpacing: 1
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                }
                Item { width: 4 }
                MotionButton {
                    id: faultsBtn
                    text: "CLEAR ERROR"
                    Layout.preferredWidth: 130; Layout.preferredHeight: 50
                    hoverScale: 1.012
                    pressScale: 0.99
                    font.pixelSize: 13; font.bold: true
                    onClicked: cartridgeController.resetFaults()
                    background: Rectangle {
                        radius: 6
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: faultsBtn.pressed ? Qt.darker(root.cBtnClearStart, 1.15) : root.cBtnClearStart }
                            GradientStop { position: 1.0; color: faultsBtn.pressed ? Qt.darker(root.cBtnClearEnd, 1.15) : root.cBtnClearEnd }
                        }
                        border.color: root.cBtnClearEnd; border.width: 1
                        Behavior on color { ColorAnimation { duration: 120 } }
                    }
                    contentItem: Item {
                        Text {
                            anchors.fill: parent
                            text: faultsBtn.text; font: faultsBtn.font; color: root.cWhiteText
                            horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                        }
                        HoverHint {
                            visible: faultsBtn.hovered
                            label: "Clear Error"
                            bc: root.cBtnClearEnd
                            tc: root.cWhiteText
                        }
                    }
                }
                Item { width: 4 }
                MotionButton {
                    id: closeBtn
                    Layout.preferredWidth: 50; Layout.preferredHeight: 50
                    hoverScale: 1.012
                    pressScale: 0.99
                    onClicked: Qt.quit()
                    background: Rectangle {
                        radius: 6
                        color: "transparent"
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: closeBtn.pressed ? Qt.darker(root.cBtnDangerStart, 1.15) : root.cBtnDangerStart }
                            GradientStop { position: 1.0; color: closeBtn.pressed ? Qt.darker(root.cBtnDangerEnd, 1.15) : root.cBtnDangerEnd }
                        }
                        border.color: root.cBtnDangerEnd
                        border.width: 1
                        Behavior on color { ColorAnimation { duration: 120 } }
                        Behavior on border.color { ColorAnimation { duration: 120 } }
                    }
                    contentItem: Item {
                        Image {
                            source: "qrc:/icons/qml/icons/power_settings.svg"
                            width: 30; height: 30
                            anchors.centerIn: parent
                            fillMode: Image.PreserveAspectFit; smooth: true
                        }
                        HoverHint {
                            visible: closeBtn.hovered
                            label: "Tắt giao diện"
                            bc: root.cBtnDangerEnd
                            tc: root.cWhiteText
                        }
                    }
                }
                Item { width: 4 }
            }

            Text {
                anchors.centerIn: parent
                text: "ROS2 - CARTRIDGE PROVISION SYSTEM"
                color: root.cAccent
                font.pixelSize: 24; font.bold: true; font.letterSpacing: 1.5
                z: 11
            }
        }

        // ════════════════════════════════════════════════════════════
        // OUTPUT TRAY TIMEOUT WARNING
        // ════════════════════════════════════════════════════════════

        Popup {
            id: outputWarningPopup
            width: 320; height: 160
            anchors.centerIn: parent
            modal: true; focus: true
            closePolicy: Popup.NoAutoClose
            background: Rectangle {
                color: "#140d05"
                border.color: root.cOrange
                border.width: 2
                radius: 10
            }
            contentItem: ColumnLayout {
                spacing: 15
                Text {
                    text: "⚠️ CẢNH BÁO"
                    color: root.cWhiteText
                    font.pixelSize: 18; font.bold: true
                    Layout.alignment: Qt.AlignHCenter
                }
                Text {
                    text: "Hệ thống đang chờ khay cấp khay thành phẩm.\ Bạn đã cấp khay mới chưa?"
                    color: root.cWhiteText
                    font.pixelSize: 14
                    horizontalAlignment: Text.AlignHCenter
                    Layout.alignment: Qt.AlignHCenter
                }
                Item { Layout.fillHeight: true }
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 20
                    MotionButton {
                        text: "ĐÃ CẤP KHAY"
                        font.bold: true; font.pixelSize: 12
                        Layout.preferredWidth: 120; Layout.preferredHeight: 35
                        background: Rectangle {
                            radius: 8
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: root.cBtnPrimaryStart }
                                GradientStop { position: 1.0; color: root.cBtnPrimaryEnd }
                            }
                            border.color: root.cBtnPrimaryBorder
                        }
                        contentItem: Text { text: parent.text; font: parent.font; color: root.cWhiteText; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: {
                            cartridgeController.confirmOutput();
                            // outputWarningPopup.close();
                        }
                    }
                    MotionButton {
                        text: "CHỜ THÊM"
                        font.bold: true; font.pixelSize: 12
                        Layout.preferredWidth: 100; Layout.preferredHeight: 35
                        background: Rectangle {
                            radius: 8
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: root.cBtnBaseStart }
                                GradientStop { position: 1.0; color: root.cBtnBaseEnd }
                            }
                            border.color: root.cBtnBaseBorder
                            border.width: 1
                        }
                        contentItem: Text { text: parent.text; font: parent.font; color: root.cWhiteText; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: {
                            // outputWarningPopup.close();
                        }
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // NOTIFICATION BANNER — gui_notify từ node (watchdog, cylinder timeout, errors)
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: notifyBanner
            anchors { top: header.bottom; left: parent.left; right: parent.right }
            height: visible ? 36 : 0
            visible: false
            z: 9
            clip: true
            Behavior on height { NumberAnimation { duration: 200 } }

            property string lvl: "info"   // "info" | "warn" | "error"
            property string ttl: ""
            property string dtl: ""

            color: {
                if (lvl === "error") return "#2a0808"
                if (lvl === "warn")  return "#221808"
                return "#081627"
            }
            border.color: {
                if (lvl === "error") return root.cRed
                if (lvl === "warn")  return root.cOrange
                return root.cCyan
            }
            border.width: 1

            Connections {
                target: cartridgeController
                function onNotificationReceived() {
                    try {
                        var obj = JSON.parse(cartridgeController.lastNotification)
                        if (obj.level && obj.level.indexOf("silent") === 0) return;
                        // Banner tắt theo yêu cầu — mọi thông báo xem trong Activity Log.
                        // notifyBanner.lvl = obj.level  || "info"
                        // notifyBanner.ttl = obj.title  || ""
                        // notifyBanner.dtl = obj.detail || ""
                        // notifyBanner.visible = true
                        // bannerTimer.restart()

                        if (obj.title === "Tray detected") {
                            // outputWarningPopup.open()
                        }
                    } catch(e) {}
                }
            }

            Timer {
                id: bannerTimer
                // info/warn tự ẩn sau 6s — error ở lại cho đến khi bấm ✕
                interval: notifyBanner.lvl === "error" ? 30000 : 6000
                onTriggered: { if (notifyBanner.lvl !== "error") notifyBanner.visible = false }
            }

            RowLayout {
                anchors { fill: parent; leftMargin: 12; rightMargin: 6 }
                spacing: 8

                // Level icon
                Text {
                    text: {
                        if (notifyBanner.lvl === "error") return "🚨"
                        if (notifyBanner.lvl === "warn")  return "⚠"
                        return "ℹ"
                    }
                    color: root.cWhiteText
                    font.pixelSize: 13
                }
                // Title
                Text {
                    text: notifyBanner.ttl
                    color: root.cWhiteText
                    font.pixelSize: 12; font.bold: true
                }
                // Detail (fills remaining space)
                Text {
                    text: notifyBanner.dtl
                    color: root.cWhiteText; font.pixelSize: 11
                    Layout.fillWidth: true
                    elide: Text.ElideRight; opacity: 0.9
                }
                // Reset Faults shortcut (chỉ hiện khi error)
                MotionButton {
                    visible: notifyBanner.lvl === "error"
                    text: "Reset"
                    Layout.preferredHeight: 22
                    font.pixelSize: 10; font.bold: true
                    onClicked: { cartridgeController.resetFaults(); notifyBanner.visible = false }
                    background: Rectangle {
                        radius: 6
                        color: root.cBtnWarningEnd
                        border.color: root.cBtnWarningBorder
                        border.width: 1
                    }
                    contentItem: Text { text: parent.text; font: parent.font; color: root.cWhiteText;
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                // Dismiss button
                MotionButton {
                    text: "✕"
                    Layout.preferredWidth: 22; Layout.preferredHeight: 22
                    font.pixelSize: 11
                    onClicked: notifyBanner.visible = false
                    background: Rectangle { radius: 6; color: root.cBtnBaseEnd; border.color: root.cBtnBaseBorder; border.width: 1 }
                    contentItem: Text { text: parent.text; font: parent.font; color: root.cWhiteText;
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // TAB BAR
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: tabbar
            anchors { bottom: parent.bottom; left: parent.left; right: parent.right }
            anchors.leftMargin: 16
            anchors.rightMargin: 16
            anchors.bottomMargin: root.tabbarDockGap
            height: root.tabbarH
            radius: height / 2
            antialiasing: true
            z: 20
            gradient: Gradient {
                orientation: Gradient.Vertical
                GradientStop { position: 0.0; color: Qt.rgba(0.12, 0.18, 0.26, 0.40) }
                GradientStop { position: 0.42; color: root.cDashPanel }
                GradientStop { position: 1.0; color: Qt.rgba(0.04, 0.07, 0.11, 0.30) }
            }
            border.color: Qt.rgba(1, 1, 1, 0.12)
            border.width: 1
            layer.enabled: true
            layer.effect: DropShadow {
                transparentBorder: true
                radius: 34
                samples: 41
                horizontalOffset: 0
                verticalOffset: 11
                color: "#2d39516e"
            }

            // drag-to-switch: press anywhere on tabbar and swipe left/right
            property real _dragPressX: 0
            property real _dragCurrentX: 0
            property bool _wasDrag: false
            MotionMouseArea {
                id: tabDragArea
                anchors.fill: parent
                z: 10
                motionEnabled: false
                shadowEnabled: false
                shimmerEnabled: false
                onPressed: {
                    tabbar._dragPressX = mouseX
                    tabbar._dragCurrentX = mouseX
                    tabbar._wasDrag = false
                }
                onPositionChanged: {
                    tabbar._dragCurrentX = mouseX
                    if (Math.abs(mouseX - tabbar._dragPressX) > 30)
                        tabbar._wasDrag = true
                }
                onReleased: {
                    if (tabbar._wasDrag) {
                        var dx = mouseX - tabbar._dragPressX
                        if (dx < 0 && stack.currentIndex < 5) root.setStackIndex(stack.currentIndex + 1)
                        else if (dx > 0 && stack.currentIndex > 0) root.setStackIndex(stack.currentIndex - 1)
                    } else {
                        var tabWidth = (tabbar.width - 8) / 6
                        var clickedIndex = Math.floor(Math.max(0, Math.min(tabbar.width - 9, mouseX - 4)) / tabWidth)
                        root.setStackIndex(clickedIndex)
                    }
                    tabbar._dragCurrentX = mouseX
                }
                onCanceled: {
                    tabbar._wasDrag = false
                    tabbar._dragCurrentX = tabbar._dragPressX
                }
            }

            Rectangle {
                id: tabGrip
                width: Math.max(74, (tabbar.width - 8) / 6 - 12)
                height: tabbar.height - 14
                radius: height / 2
                y: 7
                property real tabWidth: (tabbar.width - 8) / 6
                property real dragPreviewOffset: tabDragArea.pressed && tabbar._wasDrag
                                                 ? Math.max(-tabWidth, Math.min(tabWidth, tabbar._dragCurrentX - tabbar._dragPressX)) * 0.28
                                                 : 0
                x: 4 + stack.currentIndex * tabWidth + (tabWidth - width) / 2 + dragPreviewOffset
                z: 0
                gradient: Gradient {
                    orientation: Gradient.Vertical
                    GradientStop { position: 0.0; color: root.cTabSelectedTop }
                    GradientStop { position: 0.54; color: root.cTabSelectedMid }
                    GradientStop { position: 1.0; color: root.cTabSelectedBottom }
                }
                border.color: root.cTabSelectedBorder
                border.width: 1
                opacity: 1.0
                scale: tabDragArea.pressed ? 1.01 : 1.0
                transformOrigin: Item.Center
                layer.enabled: true
                layer.effect: DropShadow {
                    transparentBorder: true
                    radius: 18
                    samples: 27
                    horizontalOffset: 0
                    verticalOffset: 5
                    color: "#36485d9a"
                }
                Behavior on x { NumberAnimation { duration: 220; easing.type: Easing.OutCubic } }
                Behavior on opacity { NumberAnimation { duration: 120 } }
                Behavior on scale { NumberAnimation { duration: 120; easing.type: Easing.OutQuad } }
            }

            Row {
                anchors.fill: parent
                anchors.margins: 4
                spacing: 0
                Repeater {
                    model: ListModel {
                        ListElement { t: "Control Dashboard"; k: "control" }
                        ListElement { t: "Technical System";  k: "config"  }
                        ListElement { t: "Robot Control";     k: "robot"   }
                        ListElement { t: "Fill HP Control";   k: "hp"      }
                        ListElement { t: "Ink System";        k: "ink"     }
                        ListElement { t: "Production Output"; k: "prod"    }
                    }
                    delegate: MotionButton {
                        id: tabButton
                        property bool isSelected: stack.currentIndex === index
                        property bool hasIcon: true
                        property string tabIconSource: model.k === "robot"
                                                        ? "qrc:/icons/qml/icons/bot.svg"
                                                        : (model.k === "ink"
                                                            ? "qrc:/icons/qml/icons/droplet.svg"
                                                            : (model.k === "config"
                                                               ? "qrc:/icons/qml/icons/settings.svg"
                                                               : (model.k === "control"
                                                                  ? "qrc:/icons/qml/icons/monitor_cog.svg"
                                                                  : (model.k === "hp"
                                                                     ? "qrc:/icons/qml/icons/file_sliders.svg"
                                                                     : (model.k === "prod"
                                                                        ? "qrc:/icons/qml/icons/monitor_cloud.svg"
                                                                        : "")))))
                        height: parent.height
                        width: (tabbar.width - 8) / 6
                        z: 1
                        hoverScale: 1.03
                        pressScale: 0.98
                        shadowColor: "#66000000"
                        onClicked: if (!tabbar._wasDrag) root.setStackIndex(index)

                        background: Rectangle {
                            radius: height / 2
                            antialiasing: true
                            color: tabButton.pressed ? root.pressColor("#15ffffff") : (tabButton.hovered && !tabButton.isSelected ? "#15ffffff" : "transparent")
                            border.color: tabButton.isSelected ? "#00ffffff" : (tabButton.hovered ? "#34d8eef5" : "transparent")
                            border.width: tabButton.hovered && !tabButton.isSelected ? 1 : 0
                        }

                        contentItem: Item {
                            anchors.fill: parent

                            Item {
                                id: tabIconHost
                                width: 36
                                height: 36
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.top: parent.top
                                anchors.topMargin: 1

                                Image {
                                    id: tabIconImage
                                    visible: false
                                    source: tabButton.tabIconSource
                                    width: 34
                                    height: 34
                                    anchors.centerIn: parent
                                    fillMode: Image.PreserveAspectFit
                                }
                                ColorOverlay {
                                    anchors.fill: tabIconImage
                                    source: tabIconImage
                                    color: tabButton.isSelected ? root.cTabSelectedContent : root.cWhiteText
                                }
                            }

                            Text {
                                text: model.t
                                font.pixelSize: 14
                                font.bold: true
                                font.letterSpacing: 0
                                color: tabButton.isSelected ? root.cTabSelectedContent : root.cWhiteText
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                width: tabButton.width - 4
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.top: tabIconHost.bottom
                                anchors.topMargin: -2
                                wrapMode: Text.NoWrap
                                elide: Text.ElideNone
                            }
                        }
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // STACK
        // ════════════════════════════════════════════════════════════
        StackLayout {
            id: stack
            anchors { top: notifyBanner.bottom; left: parent.left; right: parent.right; bottom: tabbar.top }
            currentIndex: 0
            transform: Translate { id: stackSlide; x: 0 }

            DragHandler {
                id: screenSwipeHandler
                target: null
                acceptedButtons: Qt.LeftButton
                grabPermissions: PointerHandler.CanTakeOverFromAnything
                xAxis.enabled: true
                yAxis.enabled: false

                onActiveChanged: {
                    if (active) {
                        root.screenDragStartIndex = stack.currentIndex
                    } else {
                        var dx = activeTranslation.x
                        if (Math.abs(dx) > 90) {
                            if (dx < 0 && root.screenDragStartIndex < 5)
                                root.setStackIndex(root.screenDragStartIndex + 1)
                            else if (dx > 0 && root.screenDragStartIndex > 0)
                                root.setStackIndex(root.screenDragStartIndex - 1)
                        }
                    }
                }
            }

            ParallelAnimation {
                id: stackSlideAnim
                NumberAnimation {
                    target: stackSlide
                    property: "x"
                    to: 0
                    duration: 240
                    easing.type: Easing.OutCubic
                }
                NumberAnimation {
                    target: stack
                    property: "opacity"
                    to: 1.0
                    duration: 180
                    easing.type: Easing.OutQuad
                }
            }

            // ── PAGE 1: CONTROL DASHBOARD ────────────────────────
            Item {
                Item {
                    id: pageGrid
                    anchors { fill: parent; margins: root.pad }

                    // ─ TOP CARDS ROW ─────────────────────────────
                    RowLayout {
                        id: topCardsRow
                        x: 0; y: 0
                        width: parent.width - root.sensorW - root.gap
                        height: 260
                        spacing: root.gap

                        // ── Mode Selection ──────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.166
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cControlBorder }
                            GlassHighlight {}

                            ColumnLayout { id: modeSelCol
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4

                                // modeBlocked: đang chạy (state machine busy) HOẶC chưa chọn mode (idle)
                                property bool modeIsIdle: root.currentUiMode === "idle" || root.currentUiMode === ""
                                property bool modeBlocked: {
                                    var s = cartridgeController.systemState.toLowerCase()
                                    return s !== "" && s !== "idle" && s !== "unknown"
                                }

                                Text {
                                    text: "MODE SELECTION"; color: root.cCardTitle
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // ── Mode Items (always visible, no dropdown) ──
                                Column {
                                    Layout.fillWidth: true
                                    Layout.fillHeight: true
                                    spacing: 6

                                    // AUTO
                                    Rectangle {
                                        id: autoModeRect
                                        width: parent.width
                                        height: (parent.height - 12) / 3
                                        radius: 8
                                        property bool isModeSelected: root.currentUiMode === "auto"
                                        color: "transparent"
                                        border.color: isModeSelected ? root.cModeSelectedBorder : root.cProvisionButtonBorder
                                        border.width: isModeSelected ? 2 : 1
                                        Behavior on color { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        Rectangle {
                                            anchors.fill: parent
                                            radius: parent.radius
                                            visible: autoModeRect.isModeSelected || autoModeMA.pressed
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: autoModeMA.pressed ? root.pressGradientColor(root.cModeSelectedTop) : root.cModeSelectedTop }
                                                GradientStop { position: 0.54; color: autoModeMA.pressed ? root.pressGradientColor(root.cModeSelectedMid) : root.cModeSelectedMid }
                                                GradientStop { position: 1.0; color: autoModeMA.pressed ? root.pressGradientColor(root.cModeSelectedBottom) : root.cModeSelectedBottom }
                                            }
                                        }
                                        MotionMouseArea {
                                            id: autoModeMA
                                            anchors.fill: parent
                                            enabled: !modeSelCol.modeBlocked
                                            hoverScale: 1.02
                                            pressScale: 0.976
                                            shadowEnabled: false
                                            shimmerEnabled: false
                                            onClicked: {
                                                mainWindow.syncOperationMode("auto")
                                            }
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 2
                                            Text { text: "AUTO"; color: autoModeRect.isModeSelected ? root.cModeSelectedText : root.cWhiteText; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                            Text { text: "Automatic"; color: autoModeRect.isModeSelected ? root.cModeSelectedText : root.cWhiteText; font.pixelSize: 9; anchors.horizontalCenter: parent.horizontalCenter }
                                        }
                                    }

                                    // AI MODE
                                    Rectangle {
                                        id: aiModeRect
                                        width: parent.width
                                        height: (parent.height - 12) / 3
                                        radius: 8
                                        property bool isModeSelected: root.currentUiMode === "ai"
                                        color: "transparent"
                                        border.color: isModeSelected ? root.cModeSelectedBorder : root.cProvisionButtonBorder
                                        border.width: isModeSelected ? 2 : 1
                                        Behavior on color { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        Rectangle {
                                            anchors.fill: parent
                                            radius: parent.radius
                                            visible: aiModeRect.isModeSelected || aiModeMA.pressed
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: aiModeMA.pressed ? root.pressGradientColor(root.cModeSelectedTop) : root.cModeSelectedTop }
                                                GradientStop { position: 0.54; color: aiModeMA.pressed ? root.pressGradientColor(root.cModeSelectedMid) : root.cModeSelectedMid }
                                                GradientStop { position: 1.0; color: aiModeMA.pressed ? root.pressGradientColor(root.cModeSelectedBottom) : root.cModeSelectedBottom }
                                            }
                                        }
                                        MotionMouseArea {
                                            id: aiModeMA
                                            anchors.fill: parent
                                            enabled: !modeSelCol.modeBlocked
                                            hoverScale: 1.02
                                            pressScale: 0.976
                                            shadowEnabled: false
                                            shimmerEnabled: false
                                            onClicked: {
                                                mainWindow.syncOperationMode("ai")
                                            }
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 2
                                            Text { text: "AI MODE"; color: aiModeRect.isModeSelected ? root.cModeSelectedText : root.cWhiteText; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                            Text { text: "Camera AI"; color: aiModeRect.isModeSelected ? root.cModeSelectedText : root.cWhiteText; font.pixelSize: 9; anchors.horizontalCenter: parent.horizontalCenter }
                                        }
                                    }

                                    // MANUAL
                                    Rectangle {
                                        id: manualModeRect
                                        width: parent.width
                                        height: (parent.height - 12) / 3
                                        radius: 8
                                        property bool isModeSelected: root.currentUiMode === "manual" || root.currentUiMode === "jog"
                                        color: "transparent"
                                        border.color: isModeSelected ? root.cModeSelectedBorder : root.cProvisionButtonBorder
                                        border.width: isModeSelected ? 2 : 1
                                        Behavior on color { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        Rectangle {
                                            anchors.fill: parent
                                            radius: parent.radius
                                            visible: manualModeRect.isModeSelected || manualModeMA.pressed
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: manualModeMA.pressed ? root.pressGradientColor(root.cModeSelectedTop) : root.cModeSelectedTop }
                                                GradientStop { position: 0.54; color: manualModeMA.pressed ? root.pressGradientColor(root.cModeSelectedMid) : root.cModeSelectedMid }
                                                GradientStop { position: 1.0; color: manualModeMA.pressed ? root.pressGradientColor(root.cModeSelectedBottom) : root.cModeSelectedBottom }
                                            }
                                        }
                                        MotionMouseArea {
                                            id: manualModeMA
                                            anchors.fill: parent
                                            enabled: !modeSelCol.modeBlocked
                                            hoverScale: 1.02
                                            pressScale: 0.976
                                            shadowEnabled: false
                                            shimmerEnabled: false
                                            onClicked: {
                                                mainWindow.syncOperationMode("manual")
                                            }
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 2
                                            Text { text: "MANUAL"; color: manualModeRect.isModeSelected ? root.cModeSelectedText : root.cWhiteText; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                            Text { text: "Direct Control"; color: manualModeRect.isModeSelected ? root.cModeSelectedText : root.cWhiteText; font.pixelSize: 9; anchors.horizontalCenter: parent.horizontalCenter }
                                        }
                                    }
                                }
                            }
                        }

                        // ── System Control ───────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.208
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cControlBorder }
                            GlassHighlight {}

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4

                                Text {
                                    text: "SYSTEM CONTROL"; color: root.cCardTitle
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // 2 columns x 2 rows — same GridLayout structure as other cards
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 2; columnSpacing: 4; rowSpacing: 4

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "START"; iconSource: "qrc:/qml/icons/play.svg"; bg: root.cBtnPrimaryStart; bgEnd: root.cBtnPrimaryEnd; bc: root.cBtnPrimaryBorder; tc: "#ffffff"; clickEnabled: !root.startCommandLocked; onClicked: {
                                            if (root.startCommandLocked)
                                                return
                                            root.startCommandLocked = true
                                            mainWindow.startSynchronizedSystems(root.currentUiMode)
                                        } }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "RESUME"; iconSource: "qrc:/qml/icons/step_forward.svg"; bg: root.cServoRunStart; bgEnd: root.cServoRunEnd; bc: root.cServoRunBorder; tc: root.cServoRunText; onClicked: { root.pauseLatched = false; cartridgeController.resumeSystem() } }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STOP"; bg: root.cBtnDangerStart; bgEnd: root.cBtnDangerEnd; bc: root.cBtnDangerBorder; tc: "#ffffff"; blinking: cartridgeController.uiHint === "press_stop"; onClicked: root.stopFromSystemControl() }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "PAUSE"; bg: root.cBtnWarningStart; bgEnd: root.cBtnWarningEnd; selectedBg: root.cBtnWarningStart; selectedBgEnd: root.cBtnWarningEnd; bc: root.cBtnWarningBorder; tc: "#ffffff"; selectedTc: "#ffffff"; isSelected: root.pauseLatched; onClicked: { root.pauseLatched = true; cartridgeController.pauseSystem() } }
                                }
                            }
                        }

                        // ── State Navigation ─────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.281
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cControlBorder }
                            GlassHighlight {}

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4
                                property bool stateNavLockedByAutoAiRun: mainWindow.autoAiStartedSinceModeSelect
                                                                     && (root.currentUiMode === "auto"
                                                                         || root.currentUiMode === "ai")
                                property bool stateNavEnabled: !modeSelCol.modeIsIdle && !stateNavLockedByAutoAiRun
                                // Không cho chạy khi chưa chọn mode hoặc AUTO/AI đã START.
                                enabled: stateNavEnabled
                                opacity: stateNavEnabled ? 1.0 : 0.35
                                Behavior on opacity { NumberAnimation { duration: 200 } }

                                Text {
                                    text: "STATE NAVIGATION"; color: root.cCardTitle
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // 3 columns x 2 rows square grid
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 3; columnSpacing: 4; rowSpacing: 4

                                    CBtn {
                                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1
                                        lbl: "HOMING"
                                        iconSource: "qrc:/qml/icons/house.svg"
                                        bg: root.cBtnHomingStart
                                        bgEnd: root.cBtnHomingEnd
                                        bc: root.cBtnHomingBorder
                                        tc: root.cBtnHomingText
                                        isSelected: root.homingBusy()
                                        clickEnabled: !root.homingBusy()
                                        blinking: cartridgeController.uiHint === "press_homing"
                                        onClicked: {
                                            root.homingCommandLocked = true
                                            homingLockFailsafeTimer.restart()
                                            robotController.gotoState("HOMING")
                                            cartridgeController.gotoState("HOMING")
                                        }
                                    }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 1\nFEEDING TRAY\nPOS1"; labelHorizontalAlignment: Text.AlignHCenter; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cBtnPrimaryStart; selectedBgEnd: root.cBtnPrimaryEnd; bc: root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: root.state1Active(); clickEnabled: !root.state1Active(); glassStyle: isSelected; onClicked: cartridgeController.gotoState("STATE1") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 3\nFEEDING TRAY\nPOS2"; labelHorizontalAlignment: Text.AlignHCenter; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cBtnPrimaryStart; selectedBgEnd: root.cBtnPrimaryEnd; bc: root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: root.state3Active(); clickEnabled: !root.state3Active(); glassStyle: isSelected; onClicked: cartridgeController.gotoState("STATE3") }

                                    CBtn {
                                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1
                                        lbl: root.currentUiMode === "jog" ? "STATE MODE" : "JOG MODE"
                                        bg: root.cServoRunStart
                                        bgEnd: root.cServoRunEnd
                                        bc: root.cServoRunBorder
                                        tc: root.cServoRunText
                                        glassStyle: false
                                        isSelected: root.currentUiMode === "jog" || cartridgeController.systemState.toLowerCase().indexOf("jog") !== -1
                                        onClicked: {
                                            root.jogStopStateHint = false
                                            if (root.cartridgeStateActive()) {
                                                root.abortStateToJog()
                                                return
                                            }
                                            if (root.currentUiMode === "jog") {
                                                root.suppressJogEchoForManual = true
                                                mainWindow.syncOperationMode("manual")
                                            } else {
                                                mainWindow.syncOperationMode("jog")
                                            }
                                        }
                                    }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 2\nREPLACE TRAY\nPOS1"; labelHorizontalAlignment: Text.AlignHCenter; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cBtnPrimaryStart; selectedBgEnd: root.cBtnPrimaryEnd; bc: root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: root.state2Active(); clickEnabled: !root.state2Active(); glassStyle: isSelected; onClicked: cartridgeController.gotoState("STATE2") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 4\nREPLACE TRAY\nPOS2"; labelHorizontalAlignment: Text.AlignHCenter; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cBtnPrimaryStart; selectedBgEnd: root.cBtnPrimaryEnd; bc: root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: root.state4Active(); clickEnabled: !root.state4Active(); glassStyle: isSelected; onClicked: cartridgeController.gotoState("STATE4") }
                                }
                            }
                        }

                        // ── Control Cylinder ──────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.345
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cControlBorder }
                            GlassHighlight {}

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4
                                property bool cylEnabled: cartridgeController.currentMode === "jog"
                                enabled: cylEnabled
                                opacity: cylEnabled ? 1.0 : 0.35
                                Behavior on opacity { NumberAnimation { duration: 200 } }

                                Text {
                                    text: "CONTROL CYLINDER"; color: root.cCardTitle
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // 5 columns x 2 rows: Cyl1, Cyl2, Cyl3, Cyl4, Cyl5
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 5; columnSpacing: 4; rowSpacing: 4

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL1\nPICKTRAY1\nEXTEND"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 10 && cartridgeController.sensorState.charAt(9) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(1, true) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL2\nPICKTRAY2\nEXTEND"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 22 && cartridgeController.sensorState.charAt(21) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(2, true) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL3\nHOLDOUT1\nEXTEND"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 16 && cartridgeController.sensorState.charAt(15) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(3, true) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL4\nHOLDIN2\nEXTEND"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 26 && cartridgeController.sensorState.charAt(25) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(4, true) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL5\nHOLDOUT2\nEXTEND"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 28 && cartridgeController.sensorState.charAt(27) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(5, true) }

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL1\nPICKTRAY1\nRETRACT"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 9 && cartridgeController.sensorState.charAt(8) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(1, false) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL2\nPICKTRAY2\nRETRACT"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 21 && cartridgeController.sensorState.charAt(20) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(2, false) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL3\nHOLDOUT1\nRETRACT"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 15 && cartridgeController.sensorState.charAt(14) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(3, false) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL4\nHOLDIN2\nRETRACT"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 25 && cartridgeController.sensorState.charAt(24) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(4, false) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; radius: 8; lbl: "CYL5\nHOLDOUT2\nRETRACT"; bg: "transparent"; bgEnd: "transparent"; selectedBg: root.cCylinderActiveStart; selectedBgEnd: root.cCylinderActiveEnd; selectedTc: root.cCylinderActiveText; bc: isSelected ? "transparent" : root.cBtnBaseBorder; tc: root.cBtnBaseText; isSelected: cartridgeController.sensorState.length >= 27 && cartridgeController.sensorState.charAt(26) === '1'; glassStyle: isSelected; selectedInset: false; onClicked: cartridgeController.cylinderCmd(5, false) }
                                }
                            }
                        }
                    }
                    // ─ SERVO CONTROL AREA ────────────────────────
                    Rectangle {
                        x: 0
                        y: topCardsRow.height + root.gap
                        width: parent.width - root.sensorW - root.gap
                        height: root.topH - topCardsRow.height - root.gap
                        color: root.cControlPanel; border.color: root.cControlBorder; radius: 6; clip: true
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cControlBorder }

                        Column {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4

                            Row { width: parent.width; height: 24; spacing: 6
                                Text { text: "SERVO CONTROL"; color: root.cCardTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5; anchors.verticalCenter: parent.verticalCenter }
                                // hidden data source — syncs jogVelMms from FAS PNU via _jog_vel topic
                                Item {
                                    id: velDisplay
                                    visible: false; width: 0; height: 0
                                    property int jogVelMms: 30
                                    Connections {
                                        target: cartridgeController
                                        function onServoPositionsChanged() {
                                            try {
                                                var d = JSON.parse(cartridgeController.servoPositions)
                                                if (d["_jog_vel"] !== undefined)
                                                    velDisplay.jogVelMms = Math.round(Number(d["_jog_vel"]) * 1000)
                                            } catch(e) {}
                                        }
                                    }
                                }
                            }

                            // 5 servo cards horizontal
                            Row {
                                id: servoRow
                                width: parent.width
                                height: parent.height - 20 - 4
                                spacing: root.gap
                                property bool jogAllowed: cartridgeController.currentMode === "jog"

                                Repeater {
                                    model: ListModel {
                                        ListElement { sid: 1; sname: "InX";     sdesc: "Trục X đầu vào" }
                                        ListElement { sid: 2; sname: "InY";     sdesc: "Trục Y đầu vào" }
                                        ListElement { sid: 3; sname: "PutTray"; sdesc: "Đẩy khay" }
                                        ListElement { sid: 4; sname: "OutX";    sdesc: "Trục X đầu ra" }
                                        ListElement { sid: 5; sname: "OutY";    sdesc: "Trục Y đầu ra" }
                                    }
                                    delegate: Rectangle {
                                        id: cardItem
                                        property int jogVelMms: 30
                                        readonly property int controlH: Math.max(36, Math.min(40, Math.floor((height - 170) / 5)))
                                        Connections {
                                            target: cartridgeController
                                            function onServoPositionsChanged() {
                                                try {
                                                    var fv = JSON.parse(cartridgeController.servoPositions)["_fas_vel"]
                                                    if (fv && fv[String(model.sid)] !== undefined)
                                                        cardItem.jogVelMms = Math.round(Number(fv[String(model.sid)]) * 1000)
                                                } catch(e) {}
                                            }
                                        }
                                        width: Math.floor((servoRow.width - 4*root.gap) / 5)
                                        height: servoRow.height
                                        color: root.cCard; border.color: root.cBorder; radius: 6; clip: true
                                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cDashCardBorderHover : root.cDashCardBorder }

                                        Column {
                                            anchors.fill: parent
                                            anchors.margins: 6
                                            spacing: 6; width: parent.width - 12

                                            // header: name + desc
                                            Column { width: parent.width; spacing: 2
                                                Text { text: "S"+model.sid+": "+model.sname; color: root.cCardTitle; font.pixelSize: 20; font.bold: true; width: parent.width; horizontalAlignment: Text.AlignHCenter }
                                                Text { text: model.sdesc; color: root.cWhiteText; font.pixelSize: 17; width: parent.width; horizontalAlignment: Text.AlignHCenter; elide: Text.ElideRight }
                                            }

                                            // position display — direct connect + deadband
                                            Text {
                                                id: posText
                                                width: parent.width; horizontalAlignment: Text.AlignHCenter
                                                text: "--"
                                                color: root.cWhiteText; font.pixelSize: 22; font.bold: true
                                                property real lastVal: -99999
                                                Connections {
                                                    target: cartridgeController
                                                    function onServoPositionsChanged() {
                                                        try {
                                                            var p = JSON.parse(cartridgeController.servoPositions)[model.sid]
                                                            if (p !== undefined && p !== null) {
                                                                 var v = Number(p)
                                                                 if (Math.abs(v - posText.lastVal) >= 0.05) {
                                                                     posText.lastVal = v
                                                                     posText.text = v.toFixed(1) + " mm"
                                                                 }
                                                             }
                                                         } catch(e) {}
                                                     }
                                                 }
                                            }

                                            // VELOCITY Row (aligned to left label)
                                            RowLayout {
                                                width: parent.width
                                                spacing: 8
                                                FunctionLabel {
                                                    lbl: "VELOCITY"
                                                    Layout.preferredWidth: 82
                                                    Layout.preferredHeight: cardItem.controlH
                                                }
                                                Rectangle {
                                                    Layout.fillWidth: true
                                                    Layout.preferredHeight: cardItem.controlH
                                                    radius: 6
                                                    color: "transparent"
                                                    border.color: root.cDashButtonBorder
                                                    border.width: 1
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: root.cDashButton }
                                                        GradientStop { position: 1.0; color: root.cDashButtonEnd }
                                                    }
                                                    Text {
                                                        id: velText
                                                        anchors.centerIn: parent
                                                        text: cardItem.jogVelMms > 0 ? (cardItem.jogVelMms / 1000.0).toFixed(3) + " m/s" : "–"
                                                        color: root.cFunctionFieldText
                                                        font.pixelSize: 16; font.bold: true; font.family: "monospace"
                                                    }
                                                }
                                            }

                                            // JOG Row (with - and + buttons, right-aligned and spanning width)
                                            RowLayout {
                                                width: parent.width
                                                spacing: 8
                                                FunctionLabel {
                                                    lbl: "JOG"
                                                    Layout.preferredWidth: 82
                                                    Layout.preferredHeight: cardItem.controlH
                                                }
                                                RowLayout {
                                                    Layout.fillWidth: true
                                                    spacing: 8
                                                CBtn {
                                                    iconSource: "qrc:/icons/qml/icons/jog_neg.png"
                                                    Layout.fillWidth: true; Layout.preferredWidth: 1
                                                    Layout.preferredHeight: cardItem.controlH
                                                    padV: 6; padH: 0; fontSize: 18
                                                    bg: root.cServoRunStart; bgEnd: root.cServoRunEnd; bc: root.cServoRunBorder; tc: root.cServoRunText; glassStyle: false
                                                    active: servoRow.jogAllowed
                                                    inactiveOpacity: 0.22
                                                        onPressed: {
                                                            if(servoRow.jogAllowed) {
                                                                root.showJogStopStateHint()
                                                                cartridgeController.jogServo(model.sid,"-", cardItem.jogVelMms)
                                                            }
                                                        }
                                                        onReleased: {
                                                            cartridgeController.jogStop(model.sid)
                                                            root.fadeJogStopStateHint()
                                                        }
                                                    }
                                                CBtn {
                                                    iconSource: "qrc:/icons/qml/icons/jog_plus.png"
                                                    Layout.fillWidth: true; Layout.preferredWidth: 1
                                                    Layout.preferredHeight: cardItem.controlH
                                                    padV: 6; padH: 0; fontSize: 18
                                                        bg: root.cServoRunStart; bgEnd: root.cServoRunEnd; bc: root.cServoRunBorder; tc: root.cServoRunText; glassStyle: false
                                                    active: servoRow.jogAllowed
                                                    inactiveOpacity: 0.22
                                                        onPressed: {
                                                            if(servoRow.jogAllowed) {
                                                                root.showJogStopStateHint()
                                                                cartridgeController.jogServo(model.sid,"+", cardItem.jogVelMms)
                                                            }
                                                        }
                                                        onReleased: {
                                                            cartridgeController.jogStop(model.sid)
                                                            root.fadeJogStopStateHint()
                                                        }
                                                    }
                                                }
                                            }

                                            // HOMING & CLEAR combined side-by-side to save vertical space
                                            Row {
                                                spacing: 6
                                                width: parent.width
                                                CBtn { lbl:"HOMING"; iconSource:"qrc:/qml/icons/house.svg"; w:(parent.width - 6)/2; h:cardItem.controlH; padV:6; fontSize: 16; bg:root.cServoRunStart; bgEnd:root.cServoRunEnd; bc:root.cServoRunBorder; tc:root.cServoRunText; glassStyle:false; active:servoRow.jogAllowed; inactiveOpacity:0.22; onClicked: { if(servoRow.jogAllowed) cartridgeController.homeServo(model.sid) } }
                                                CBtn { lbl:"CLEAR"; iconSource:"qrc:/qml/icons/brush_cleaning_white.svg"; w:(parent.width - 6)/2; h:cardItem.controlH; padV:6; fontSize: 16; bg:root.cServoRunStart; bgEnd:root.cServoRunEnd; bc:root.cServoRunBorder; tc:root.cServoRunText; glassStyle:false; onClicked: cartridgeController.clearServo(model.sid) }
                                            }

                                            // TARGET POSITION Row (with input & RUN button)
                                            RowLayout {
                                                width: parent.width
                                                spacing: 8
                                                FunctionLabel {
                                                    lbl: "TARGET\nPOSITION"
                                                    fontSize: 11
                                                    Layout.preferredWidth: 82
                                                    Layout.preferredHeight: cardItem.controlH
                                                }
                                                RowLayout {
                                                    Layout.fillWidth: true
                                                    spacing: 4
                                                    Rectangle {
                                                        Layout.fillWidth: true
                                                        Layout.preferredHeight: cardItem.controlH
                                                        radius: 6
                                                        color: "transparent"
                                                        border.color: posIn.activeFocus ? root.cAccent : root.cDashButtonBorder
                                                        border.width: 1
                                                        gradient: Gradient {
                                                            orientation: Gradient.Horizontal
                                                            GradientStop { position: 0.0; color: root.cDashButton }
                                                            GradientStop { position: 1.0; color: root.cDashButtonEnd }
                                                        }
                                                        SmartTextInput {
                                                            id: posIn
                                                            focusHost: root
                                                            anchors.fill: parent; anchors.margins: 4
                                                            text: "0.0"; font.pixelSize: 16; font.bold: true; font.family: "monospace"
                                                            color: root.cFunctionFieldText
                                                            horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter
                                                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                                                            validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 3 }
                                                        }
                                                    }
                                                    Text { 
                                                        text: "mm"
                                                        color: root.cWhiteText
                                                        font.pixelSize: 13
                                                        font.bold: true
                                                        Layout.alignment: Qt.AlignBottom
                                                        Layout.preferredWidth: 24
                                                        horizontalAlignment: Text.AlignHCenter 
                                                    }
                                                    CBtn {
                                                        lbl: "RUN"
                                                        Layout.preferredWidth: 80
                                                        Layout.preferredHeight: cardItem.controlH
                                                        padV: 0; fontSize: 16
                                                        bg: root.cServoRunStart; bgEnd: root.cServoRunEnd; bc: root.cServoRunBorder; tc: root.cServoRunText; glassStyle: false; active: servoRow.jogAllowed
                                                        inactiveOpacity: 0.22
                                                        onClicked: { if(servoRow.jogAllowed) { var v=parseFloat(posIn.text); if(!isNaN(v)) cartridgeController.moveServo(model.sid,v) } }
                                                    }
                                                }
                                            }

                                            // Limits display
                                            Text {
                                                text: model.sid === 1 ? "Min: -322 | Max: 560"
                                                    : (model.sid === 2 ? "Min: -80 | Max: 1025"
                                                    : (model.sid === 3 ? "Min: -25 | Max: 870"
                                                    : (model.sid === 4 ? "Min: -320 | Max: 605"
                                                    : (model.sid === 5 ? "Min: -25 | Max: 1025" : ""))))
                                                color: root.cWhiteText
                                                font.pixelSize: 13
                                                font.bold: true
                                                width: parent.width
                                                horizontalAlignment: Text.AlignHCenter
                                                visible: model.sid >= 1 && model.sid <= 5
                                            }

                                            // STOP button (full width safety button at bottom)
                                                CBtn {
                                                    lbl: "STOP"
                                                    w: parent.width; h: cardItem.controlH
                                                    padV: 6
                                                    fontSize: 18
                                                    bg: root.cBtnDangerStart
                                                    bgEnd: root.cBtnDangerEnd
                                                    bc: root.cBtnDangerBorder
                                                    tc: "#ffffff"
                                                    onClicked: mainWindow.stopSynchronizedSystems()
                                                }
                                        }
                                     }
                                 }
                             }
                         }
                    }

                    // ─ LOG AREA ──────────────────────────────────
                    Rectangle {
                        x: 0; y: root.topH + root.gap
                        width: parent.width - root.sensorW - root.gap
                        height: root.logH
                        color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                        GlassHighlight {}
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cControlBorder }

                        Column {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4
                            RowLayout { width: parent.width; height: 28
                                Text { text: "LOG ACTIVITY"; color: root.cCardTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }
                                Item { Layout.fillWidth: true }
                                CBtn { lbl:"Clear"; iconSource:"qrc:/qml/icons/brush_cleaning_white.svg"; Layout.preferredWidth: implicitWidth; Layout.preferredHeight: implicitHeight; padV:4; padH:10; fontSize: 15; bg:root.cBtnClearStart; bgEnd:root.cBtnClearEnd; bc:root.cBtnActionBorder; tc:root.cBtnClearText; onClicked: cartridgeController.clearLog() }
                            }
                            Rectangle {
                                width: parent.width; height: parent.height - 28 - 4
                                color: "#06101d"; border.color: root.cBorder; radius: 4
                                ListView { anchors { fill: parent; margins: 6 }
                                    model: cartridgeController.logEntries; clip: true; spacing: 2
                                    verticalLayoutDirection: ListView.BottomToTop
                                    delegate: Loader {
                                        property var entry: modelData
                                        active: {
                                            var m = (modelData.msg || "").toLowerCase()
                                            return m.indexOf("no error") === -1
                                        }
                                        width: parent ? parent.width : 100
                                        sourceComponent: Text {
                                            width: parent ? parent.width : 100
                                            text: "[" + entry.time + "] " + entry.msg
                                            font.pixelSize: 18; font.family: "monospace"
                                            color: {
                                                var t = entry.type || "info"
                                                if (t === "err")  return root.cRed
                                                if (t === "warn") return root.cOrange
                                                if (t === "ok")   return root.cGreen
                                                return root.cWhiteText
                                            }
                                            wrapMode: Text.WordWrap
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // ─ SENSOR SIGNALS (grid-area: servo, full height) ──
                    // Hiển thị read-only 24 sensor THẬT (S1-S10, S13-S22, S25-S28) từ IO module qua
                    // topic /providesystem/sensors_state. S11/S12 là VFD status (ATV Run/
                    // Fault) — monitor bởi vfd_logic_node, không hiển thị ở grid này. Cập
                    // nhật real-time, không click. Ô được thu nhỏ để fit vừa card.
                    Rectangle {
                        x: parent.width - root.sensorW
                        y: 0; width: root.sensorW; height: root.outerH
                        color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                        GlassHighlight {}
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cControlBorder }

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 6
                            spacing: 3

                            // ── Tiêu đề ──
                            Text {
                                text: "SENSOR SIGNAL"
                                color: root.cWhiteText; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                            }

                            // ── Status label ──
                            Text { text: "STATUS"; color: "#bfe0f5"; font.pixelSize: 10; font.bold: true; font.letterSpacing: 1 }

                            // ── Grid sensor – fillHeight để tự co vừa chiều cao còn lại ──
                            GridLayout {
                                Layout.fillWidth: true
                                Layout.fillHeight: true      // ← KEY: chiếm hết không gian còn lại
                                columns: 2
                                columnSpacing: 4
                                rowSpacing: 2

                                Repeater {
                                    model: ListModel {
                                        // [CPX 253] Module 2: I1.0–I1.7
                                        ListElement { sid:1;  slabel:"S1";  sdesc:"Belt start" }
                                        ListElement { sid:2;  slabel:"S2";  sdesc:"Belt mid" }
                                        ListElement { sid:3;  slabel:"S3";  sdesc:"Belt end" }
                                        ListElement { sid:4;  slabel:"S4";  sdesc:"Scan Stack Pos1" }
                                        ListElement { sid:5;  slabel:"S5";  sdesc:"Output det." }
                                        ListElement { sid:6;  slabel:"S6";  sdesc:"Check Tray OutP1" }
                                        ListElement { sid:7;  slabel:"S7";  sdesc:"Khay tại Robot" }
                                        ListElement { sid:8;  slabel:"S8";  sdesc:"[Reserved]" }
                                        // [CPX 253] Module 3: I2.0–I2.7
                                        // (S11/S12 ATV Run/Fault — VFD status, không hiển thị ở grid này)
                                        ListElement { sid:9;  slabel:"S9";  sdesc:"Cyl1 Ret" }
                                        ListElement { sid:10; slabel:"S10"; sdesc:"Cyl1 Ext" }
                                        ListElement { sid:13; slabel:"S13"; sdesc:"OUT1 TrayPos1" }
                                        ListElement { sid:14; slabel:"S14"; sdesc:"OUT2 TrayPos1" }
                                        ListElement { sid:15; slabel:"S15"; sdesc:"Cyl3 Ret" }
                                        ListElement { sid:16; slabel:"S16"; sdesc:"Cyl3 Ext" }
                                        // [CPX 254] Module 2: I3.0–I3.5
                                        ListElement { sid:17; slabel:"S17"; sdesc:"Platform" }
                                        ListElement { sid:18; slabel:"S18"; sdesc:"Feed OK" }
                                        ListElement { sid:19; slabel:"S19"; sdesc:"Check Tray OutP2" }
                                        ListElement { sid:20; slabel:"S20"; sdesc:"Scan Stack Pos2" }
                                        ListElement { sid:21; slabel:"S21"; sdesc:"Cyl2 Ret" }
                                        ListElement { sid:22; slabel:"S22"; sdesc:"Cyl2 Ext" }
                                        // [CPX 254] Module 4: I4.0–I4.1
                                        ListElement { sid:25; slabel:"S25"; sdesc:"Cyl4 Ret" }
                                        ListElement { sid:26; slabel:"S26"; sdesc:"Cyl4 Ext" }
                                        ListElement { sid:27; slabel:"S27"; sdesc:"Cyl5 Ret" }
                                        ListElement { sid:28; slabel:"S28"; sdesc:"Cyl5 Ext" }
                                    }
                                    delegate: Rectangle {
                                        id: sBtn
                                        property bool on_: {
                                            var st = cartridgeController.sensorState;
                                            if (model.sid > 0 && model.sid <= st.length) {
                                                return st.charAt(model.sid - 1) === '1';
                                            }
                                            return false;
                                        }

                                        Layout.fillWidth: true
                                        Layout.fillHeight: true          // ← mỗi nút chiếm đều phần chiều cao
                                        Layout.minimumHeight: 20         // ← thu nhỏ để fit đủ 20 sensor

                                        radius: 4
                                        color: "transparent"
                                        border.color: on_ ? root.cSensorActiveBorder : root.cSensorIdleBorder
                                        border.width: on_ ? 2 : 1
                                        opacity: on_ ? 1.0 : 0.78
                                        Behavior on color       { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        gradient: on_ ? sensorActiveGradient : sensorIdleGradient
                                        HoverHandler { onHoveredChanged: if(!sBtn.on_) sBtn.border.color = hovered ? Qt.rgba(root.cSensorActiveBorder.r, root.cSensorActiveBorder.g, root.cSensorActiveBorder.b, 0.45) : root.cSensorIdleBorder }
                                        Gradient {
                                            id: sensorActiveGradient
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: root.cSensorActiveStart }
                                            GradientStop { position: 1.0; color: root.cSensorActiveEnd }
                                        }
                                        Gradient {
                                            id: sensorIdleGradient
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: root.cSensorIdleBg }
                                            GradientStop { position: 1.0; color: Qt.rgba(0.02, 0.07, 0.12, 0.10) }
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 0
                                            Text {
                                                text: model.slabel
                                                color: sBtn.on_ ? root.cSensorActiveText : root.cSensorIdleText
                                                font.pixelSize: 13; font.bold: true
                                                anchors.horizontalCenter: parent.horizontalCenter
                                            }
                                            Text {
                                                text: model.sdesc
                                                color: sBtn.on_ ? root.cSensorActiveText : root.cSensorIdleText; font.pixelSize: 10; font.bold: true
                                                anchors.horizontalCenter: parent.horizontalCenter
                                                visible: model.sdesc !== ""
                                            }
                                            Rectangle {
                                                id: dotIndicator
                                                width: 4; height: 4; radius: 2
                                                color: sBtn.on_ ? root.cSensorActiveText : root.cSensorIdleDot
                                                anchors.horizontalCenter: parent.horizontalCenter

                                                Repeater {
                                                    model: 2
                                                    delegate: Rectangle {
                                                        id: ripple
                                                        anchors.centerIn: parent
                                                        width: 6; height: 6; radius: 3
                                                        color: "transparent"
                                                        border.color: root.cSensorActiveBorder
                                                        border.width: 1
                                                        opacity: 0
                                                        visible: sBtn.on_

                                                        SequentialAnimation {
                                                            running: root.continuousAnimationsEnabled && sBtn.on_
                                                            loops: Animation.Infinite
                                                            PauseAnimation { duration: index * 1000 }
                                                            ParallelAnimation {
                                                                NumberAnimation {
                                                                    target: ripple
                                                                    property: "opacity"
                                                                    from: 0.8; to: 0.0
                                                                    duration: 1500
                                                                    easing.type: Easing.OutQuad
                                                                }
                                                                NumberAnimation {
                                                                    target: ripple
                                                                    property: "scale"
                                                                    from: 1.0; to: 5.0
                                                                    duration: 1500
                                                                    easing.type: Easing.OutQuad
                                                                }
                                                            }
                                                            PauseAnimation { duration: (1 - index) * 1000 }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } // Page 1

            // ── PAGE 2: TECHNICAL SYSTEM — horizontal 4-column layout ──
            Item {
                id: page2Root
                property var parsedConfig: ({})
                property int configRevision: 0
                property var p2FilteredLog: []

                function reloadConfig() {
                    try { page2Root.parsedConfig = JSON.parse(cartridgeController.configData) } catch(e) {}
                    page2Root.configRevision++
                }

                function refreshP2Log() {
                    var filtered = []
                    var entries = cartridgeController.logEntries
                    for (var i = 0; i < entries.length; i++) {
                        var msg = (entries[i].msg || "").toLowerCase()
                        if (msg.indexOf("config") !== -1 || msg.indexOf("updated") !== -1 || msg.indexOf("saved") !== -1)
                            filtered.push(entries[i])
                    }
                    page2Root.p2FilteredLog = filtered
                }

                Connections {
                    target: cartridgeController
                    function onConfigDataChanged() { page2Root.reloadConfig() }
                    function onLogEntriesChanged() { page2Root.refreshP2Log() }
                }
                Component.onCompleted: cartridgeController.getConfig()

                // ── Layout: cards on top, log bar at bottom ──
                ColumnLayout {
                    anchors { fill: parent; margins: 10 }
                    spacing: 8

                // ── 4 cards trải đều ngang ──
                RowLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    spacing: 10

                    ConfigZoneCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 1: Input Zones (InY)"
                        configKey: "iny_input_zones"
                        configSource: page2Root.parsedConfig
                    }
                    ConfigZoneCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 1: Output Zones (InY)"
                        configKey: "iny_output_zones"
                        configSource: page2Root.parsedConfig
                    }
                    ConfigZoneCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 2: Output Zones (OutY)"
                        configKey: "outy_output_zones"
                        configSource: page2Root.parsedConfig
                    }

                    // Card 4: Servo Key Positions
                    Rectangle {
                        id: servoParamsCard
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: root.cBg2; border.color: root.cBorder; radius: 6
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                        property var servoParams: [
                            { key:"inx_home",        label:"InX Home",      desc:"S1 home" },
                            { key:"inx_target2",     label:"InX Target",    desc:"S1 lấy khay (500mm)" },
                            { key:"inx_output_stack",label:"InX OutPos",    desc:"Đặt khay output" },
                            { key:"iny_home",        label:"InY Home",      desc:"S2 home" },
                            { key:"iny_target2",     label:"InY Place",     desc:"Robot place (200mm)" },
                            { key:"iny_safe_zone",   label:"InY SafeZone",  desc:"Safe zone" },
                            { key:"servo3_target2",  label:"S3 Feed",       desc:"Feed pos (400mm)" },
                            { key:"outx_home",       label:"OutX Home",     desc:"S4 home" },
                            { key:"outx_target2",    label:"OutX Target2",  desc:"Lấy khay output" },
                            { key:"outx_target3",    label:"OutX Target3",  desc:"Đặt khay robot" },
                            { key:"outy_target1",    label:"OutY Target1",  desc:"Nâng khay (safe)" },
                            { key:"outy_pick_pos",   label:"OutY Pick",     desc:"Hạ gắp khay" },
                            { key:"target_scanoutp2",label:"OUTY TgtScan",  desc:"Điểm dừng quét S20" },
                            { key:"outy_scan_arm_mm",label:"OUTY Arm S20",  desc:"Giới hạn kích hoạt S20" }
                        ]

                        Flickable {
                            id: servoFlickable2
                            anchors { fill: parent; margins: 8 }
                            contentWidth: width; contentHeight: servoInfoCol2.height + 20
                            clip: true

                            Column {
                                id: servoInfoCol2
                                width: parent.width
                                spacing: 4

                                Text { text: "SERVO KEY POSITIONS (mm)"; color: root.cCardTitle; font.pixelSize: 18; font.bold: true; font.letterSpacing: 1.5 }
                                Row { width: parent.width; height: 24; spacing: parent.width * 0.015
                                    Repeater { model: ["Parameter","Value","Description"]
                                        delegate: Text {
                                            text: modelData
                                            color: root.cWhiteText
                                            font.pixelSize: 14; font.bold: true
                                            width: index===0 ? parent.width * 0.40 : index===1 ? parent.width * 0.22 : parent.width * 0.35
                                            height: parent.height
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                            font.capitalization: Font.AllUppercase; font.letterSpacing: 1
                                        }
                                    }
                                }
                                Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                Repeater {
                                    id: servoRepeater2
                                    model: servoParamsCard.servoParams
                                    delegate: Rectangle {
                                        required property var modelData
                                        required property int index
                                        width: servoInfoCol2.width; height: 46
                                        color: index % 2 === 0 ? "transparent" : "#06101d"
                                        property alias inputText: sInput2.text
                                        property string paramKey: modelData.key
                                        Row {
                                            anchors.verticalCenter: parent.verticalCenter
                                            width: parent.width
                                            spacing: parent.width * 0.015
                                            Text {
                                                text: modelData.label
                                                color: root.cWhiteText
                                                font { pixelSize: 18; bold: true }
                                                width: parent.width * 0.40
                                                height: parent.height
                                                verticalAlignment: Text.AlignVCenter
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                            Rectangle {
                                                width: parent.width * 0.22; height: 36; radius: 5
                                                color: "#081627"; border.color: root.cFieldBorder; border.width: 1
                                                SmartTextInput {
                                                    id: sInput2
                                                    focusHost: root
                                                    anchors { fill: parent; margins: 3 }
                                                    text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true
                                                    color: root.cWhiteText; horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter
                                                    validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 2 }
	                                                    Connections {
	                                                        function onConfigRevisionChanged() {
	                                                            var v = page2Root.parsedConfig[modelData.key]
	                                                            sInput2.text = (v !== undefined) ? String(v) : ""
	                                                        }
	                                                        target: page2Root
	                                                    }
                                                }
                                            }
                                            Text {
                                                text: modelData.desc
                                                color: root.cWhiteText
                                                font.pixelSize: 16
                                                elide: Text.ElideRight
                                                height: parent.height
                                                verticalAlignment: Text.AlignVCenter
                                                anchors.verticalCenter: parent.verticalCenter
                                                width: parent.width * 0.35
                                            }
                                        }
                                    }
                                }

                                Row { spacing: 8; topPadding: 8
                                    CBtn { lbl:"Save All"; iconSource:"icons/download.svg"; padV:10; padH:22; fontSize: 18; bg:root.cBtnPrimaryStart; bgEnd:root.cBtnPrimaryEnd; bc:root.cBtnPrimaryBorder; tc:"#ffffff"
                                        onClicked: {
                                            for (var i = 0; i < servoRepeater2.count; i++) {
                                                var item = servoRepeater2.itemAt(i)
                                                if (item && item.inputText !== "")
                                                    cartridgeController.saveConfig(item.paramKey, item.inputText)
                                            }
                                        }
                                    }
	                                    CBtn { lbl:"↺ Reset"; padV:10; padH:18; fontSize: 18; bg:root.cBtnBaseStart; bgEnd:root.cBtnBaseEnd; bc:root.cBtnBaseBorder; tc:root.cBtnBaseText; onClicked: cartridgeController.getConfig() }
                                }
                            }
                        }

                        Rectangle {
                            id: servoScroll2
                            anchors { right: parent.right; top: parent.top; bottom: parent.bottom; rightMargin: 2; topMargin: 8; bottomMargin: 8 }
                            width: 4; radius: 2; color: "#14251F"
                            visible: servoFlickable2.height < servoFlickable2.contentHeight

                            Rectangle {
                                width: parent.width; radius: 2; color: root.cAccent
                                height: Math.max(20, servoFlickable2.height * (servoFlickable2.height / servoFlickable2.contentHeight))
                                y: servoFlickable2.visibleArea.yPosition * servoFlickable2.height
                            }
                        }
                    }
                } // RowLayout

                    // ── Log bar ──────────────────────────────────────────
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 120
                        color: root.cBg2; border.color: root.cBorder; radius: 6

                        Column {
                            anchors { fill: parent; margins: 8 }
                            spacing: 4
                            RowLayout {
                                width: parent.width; height: 18
                                Text { text: "CONFIG LOG"; color: root.cCardTitle; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5 }
                                Item { Layout.fillWidth: true }
                                CBtn { lbl:"Clear"; Layout.preferredWidth: implicitWidth; Layout.preferredHeight: implicitHeight; padV:3; padH:8; fontSize: 14; bg:root.cBtnClearStart; bgEnd:root.cBtnClearEnd; bc:root.cBtnActionBorder; tc:root.cBtnClearText; onClicked: cartridgeController.clearLog() }
                            }
                            Rectangle {
                                width: parent.width; height: parent.height - 22
                                color: "#06101d"; border.color: root.cBorder; radius: 4
                                ListView {
                                    anchors { fill: parent; margins: 6 }
                                    model: page2Root.p2FilteredLog
                                    clip: true; spacing: 2
                                    verticalLayoutDirection: ListView.BottomToTop
                                    delegate: Text {
                                        width: parent ? parent.width : 100
                                        text: "[" + modelData.time + "] " + modelData.msg
                                        font.pixelSize: 12; font.family: "monospace"
                                        color: root.cWhiteText
                                        wrapMode: Text.WordWrap
                                    }
                                }
                            }
                        }
                    }

                } // ColumnLayout
            } // Page 2

            // ── PAGE 3: ROBOT CONTROL ──────────────────────────────────
            Item {
                id: page3Root
                property string currentMode: root.currentUiMode  // bind to synchronized system mode
                // MANUAL controls (JOG) stay open before START, even if AUTO / AI was selected.
                // Only lock when the chosen AUTO / AI mode has actually been started.
                property bool manualEnabled: currentMode === "jog"
                                            || currentMode === "manual"
                                            || !mainWindow.autoAiStartedSinceModeSelect
                                            || robotController.systemStatus === "IDLE"
                                            || robotController.systemStatus === "MANUAL"
                                            || robotController.systemStatus === "UNKNOWN"
                                            || robotController.systemStatus === ""
                property real stepValue: 1.0
                property int speedVal: robotController.speedRatio
                property bool rowLocked: false
                property int jogStep: 1               // 0.1, 1, 5, 10
                readonly property int titleFont: 20
                readonly property int labelFont: 14
                readonly property int buttonFont: 16
                readonly property int valueFont: 20
                readonly property int inputFont: 16
                readonly property int helperFont: 13

                Item {
                    id: p3Inner
                    anchors { fill: parent; margins: 10 }

                    // ════════════════ MODE HEADER ══════════════════════
                    Rectangle {
                        id: modeToggle
                        anchors { top: parent.top; left: parent.left; right: parent.right }
                        height: page3Root.manualEnabled ? 0 : 32
                        visible: !page3Root.manualEnabled
                        radius: 5
                        color: root.cControlPanel
                        border.color: page3Root.manualEnabled ? root.cDashButtonBorder : root.cDashCardBorder
                        border.width: 1
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: page3Root.manualEnabled ? root.cDashButton : root.cControlPanel }
                            GradientStop { position: 1.0; color: page3Root.manualEnabled ? root.cDashButtonEnd : root.cControlPanel }
                        }
                        Behavior on color { ColorAnimation { duration: 200 } }
                        Behavior on border.color { ColorAnimation { duration: 200 } }
                        Row {
                            anchors.centerIn: parent; spacing: 8
                            Text {
                                text: page3Root.manualEnabled ? "🔓" : "🔒"
                                color: root.cWhiteText
                                font.pixelSize: 14; anchors.verticalCenter: parent.verticalCenter
                            }
                            Text {
                                text: "LOCKED — " + page3Root.currentMode.toUpperCase() + " MODE"
                                color: root.cWhiteText
                                font.pixelSize: page3Root.labelFont; font.bold: true; font.letterSpacing: 1
                                anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                    }

                    // ════════════════ CONTENT AREA ════════════════════════
                    Item {
                        id: contentArea
                        anchors {
                            top: modeToggle.bottom
                            topMargin: page3Root.manualEnabled ? 0 : 8
                            left: parent.left
                            right: parent.right
                            bottom: robotLogBar.top
                            bottomMargin: 4
                        }


                        // ──────────── MANUAL MODE: JOG ────────────────────
                        Item {
                            anchors.fill: parent
                            enabled: page3Root.manualEnabled
                            opacity: page3Root.manualEnabled ? 1.0 : 0.35
                            Behavior on opacity { NumberAnimation { duration: 200 } }

                            property int colGap: 6

                            // ── 3 columns: Cartesian | Joint | IO+Controls ──
                            Row {
                                id: jogRow
                                anchors.fill: parent
                                spacing: parent.colGap

                                // ═══ CARTESIAN ═══
                                Rectangle {
                                    width: (parent.width - parent.parent.colGap * 2) * 0.38; height: parent.height
                                    color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                                    GlassHighlight {}
                                    Column {
                                        id: cartCol
                                        anchors { fill: parent; margins: 8 }
                                        spacing: 6
                                        Row { spacing: 6
                                            Rectangle { width: 4; height: 16; radius: 1; color: root.cCardTitle; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "CARTESIAN (mm)"; color: root.cCardTitle; font.pixelSize: page3Root.titleFont; font.bold: true; font.letterSpacing: 1.5 }
                                        }
                                        Repeater {
                                            id: cartRep
                                            model: [
                                                { axis: "X",  neg: "X-",  pos: "X+" },
                                                { axis: "Y",  neg: "Y-",  pos: "Y+" },
                                                { axis: "Z",  neg: "Z-",  pos: "Z+" },
                                                { axis: "RX", neg: "Rx-", pos: "Rx+" },
                                                { axis: "RY", neg: "Ry-", pos: "Ry+" },
                                                { axis: "RZ", neg: "Rz-", pos: "Rz+" }
                                            ]
                                            delegate: Row {
                                                required property var modelData
                                                required property int index
                                                width: cartCol.width; height: 52; spacing: 4
                                                Rectangle {
                                                    id: negBtn
                                                    width: 58; height: 48; radius: 8
                                                    color: "transparent"
                                                    border.color: root.cJogNegativeButtonBorder; border.width: 1
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: (negContinuousMA.pressed || negStepMA.pressed) ? root.pressGradientColor(root.cJogNegativeButton) : root.cJogNegativeButton }
                                                        GradientStop { position: 1.0; color: (negContinuousMA.pressed || negStepMA.pressed) ? root.pressGradientColor(root.cJogNegativeButtonEnd) : root.cJogNegativeButtonEnd }
                                                    }
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "-"; color: root.cWhiteText; font.pixelSize: page3Root.buttonFont; font.bold: true }
                                                    MotionMouseArea { id: negContinuousMA; anchors.fill: parent; enabled: robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart(modelData.neg); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                    MotionMouseArea { id: negStepMA; anchors.fill: parent; enabled: !robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onClicked: robotController.jogStep(modelData.neg, page3Root.stepValue) }
                                                }
                                                Rectangle {
                                                    width: parent.width - 120; height: 48; radius: 5; color: "transparent"; border.width: 1; border.color: root.cFunctionFieldBorder
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: root.cFunctionFieldStart }
                                                        GradientStop { position: 1.0; color: root.cFunctionFieldEnd }
                                                    }
                                                    Text { anchors.centerIn: parent; text: robotController.cartesianPose.length > index ? robotController.cartesianPose[index].toFixed(4) : "0.0000"; color: root.cWhiteText; font.pixelSize: page3Root.valueFont; font.bold: true }
                                                }
                                                Rectangle {
                                                    id: posBtn
                                                    width: 58; height: 48; radius: 8
                                                    color: "transparent"
                                                    border.color: root.cServoRunBorder; border.width: 1
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: (posContinuousMA.pressed || posStepMA.pressed) ? root.pressGradientColor(root.cServoRunStart) : root.cServoRunStart }
                                                        GradientStop { position: 1.0; color: (posContinuousMA.pressed || posStepMA.pressed) ? root.pressGradientColor(root.cServoRunEnd) : root.cServoRunEnd }
                                                    }
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "+"; color: root.cWhiteText; font.pixelSize: page3Root.buttonFont; font.bold: true }
                                                    MotionMouseArea { id: posContinuousMA; anchors.fill: parent; enabled: robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart(modelData.pos); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                    MotionMouseArea { id: posStepMA; anchors.fill: parent; enabled: !robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onClicked: robotController.jogStep(modelData.pos, page3Root.stepValue) }
                                                }
                                            }
                                        }
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }
                                        Row { spacing: 3; width: parent.width
                                            Repeater {
                                                id: cartInputs
                                                model: ["X","Y","Z","RX","RY","RZ"]
                                                delegate: Column { spacing: 2; width: (cartCol.width - 15) / 6
                                                    Text { text: modelData; color: root.cCardTitle; font.pixelSize: page3Root.helperFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                    Rectangle {
                                                        width: parent.width; height: 40; radius: 4; color: root.cDashCardField; border.color: root.cFunctionFieldBorder; border.width: 1
                                                        SmartTextInput { id: cartInp
                                                            focusHost: root
                                                            anchors { fill: parent; margins: 2 }
                                                            color: root.cWhiteText; font.pixelSize: page3Root.inputFont; font.bold: true; horizontalAlignment: Text.AlignHCenter; clip: true
                                                            text: robotController.cartesianPose.length > index ? robotController.cartesianPose[index].toFixed(4) : "0"
                                                            selectByMouse: true; verticalAlignment: Text.AlignVCenter; validator: DoubleValidator { notation: DoubleValidator.StandardNotation }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 8
                                                color: "transparent"; border.color: root.cGetButtonBorder
                                                border.width: 1
                                                gradient: Gradient {
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: gpMA.pressed ? root.pressGradientColor(root.cGetButton) : root.cGetButton }
                                                    GradientStop { position: 1.0; color: gpMA.pressed ? root.pressGradientColor(root.cGetButtonEnd) : root.cGetButtonEnd }
                                                }
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Row {
                                                    anchors.centerIn: parent
                                                    spacing: 6
                                                    Image {
                                                        source: "icons/hard_drive_download.svg"
                                                        width: 20; height: 20
                                                        sourceSize.width: 80
                                                        sourceSize.height: 80
                                                        fillMode: Image.PreserveAspectFit
                                                        smooth: true
                                                        mipmap: true
                                                        antialiasing: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                    Text {
                                                        text: "GET POSE"
                                                        color: root.cWhiteText
                                                        font.pixelSize: page3Root.labelFont
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                }
                                                MotionMouseArea { id: gpMA; anchors.fill: parent; hoverScale: 1.02; shadowEnabled: false; shimmerEnabled: false; onClicked: {
                                                    robotController.getPose()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.cartesianPose.length > i)
                                                            cartInputs.itemAt(i).children[1].children[0].text = robotController.cartesianPose[i].toFixed(4)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 8
                                                color: "transparent"; border.color: root.cMovJButtonBorder
                                                border.width: 1
                                                gradient: Gradient {
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: mlMA.pressed ? root.pressGradientColor(root.cMovJButton) : root.cMovJButton }
                                                    GradientStop { position: 1.0; color: mlMA.pressed ? root.pressGradientColor(root.cMovJButtonEnd) : root.cMovJButtonEnd }
                                                }
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Row {
                                                    anchors.centerIn: parent
                                                    spacing: 6
                                                    Image {
                                                        source: "icons/navigation.svg"
                                                        width: 20; height: 20
                                                        fillMode: Image.PreserveAspectFit
                                                        smooth: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                    Text {
                                                        text: "SEND MovL"
                                                        color: root.cWhiteText
                                                        font.pixelSize: page3Root.labelFont
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                }
                                                MotionMouseArea { id: mlMA; anchors.fill: parent; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: { var vals = []; for (var i = 0; i < 6; i++) { var v = parseFloat(cartInputs.itemAt(i).children[1].children[0].text); if (isNaN(v)) return; vals.push(v) } robotController.startSendMoveL(vals[0],vals[1],vals[2],vals[3],vals[4],vals[5]) } onReleased: robotController.stopSendMove(); onCanceled: robotController.stopSendMove() }
                                            }
                                        }
                                    }
                                }

                                // ═══ JOINT ═══
                                Rectangle {
                                    width: (parent.width - parent.parent.colGap * 2) * 0.38; height: parent.height
                                    color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                                    GlassHighlight {}
                                    Column {
                                        id: jointCol
                                        anchors { fill: parent; margins: 8 }
                                        spacing: 6
                                        Row { spacing: 6
                                            Rectangle { width: 4; height: 16; radius: 1; color: root.cCardTitle; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "JOINT (deg)"; color: root.cCardTitle; font.pixelSize: page3Root.titleFont; font.bold: true; font.letterSpacing: 1.5 }
                                        }
                                        Repeater {
                                            id: jointRep
                                            model: 6
                                            delegate: Row {
                                                property int jn: index + 1
                                                width: jointCol.width; height: 52; spacing: 4
                                                Rectangle {
                                                    id: jnBtn
                                                    width: 58; height: 48; radius: 8
                                                    color: "transparent"
                                                    border.color: root.cJogNegativeButtonBorder; border.width: 1
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: (jnContinuousMA.pressed || jnStepMA.pressed) ? root.pressGradientColor(root.cJogNegativeButton) : root.cJogNegativeButton }
                                                        GradientStop { position: 1.0; color: (jnContinuousMA.pressed || jnStepMA.pressed) ? root.pressGradientColor(root.cJogNegativeButtonEnd) : root.cJogNegativeButtonEnd }
                                                    }
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "-"; color: root.cWhiteText; font.pixelSize: page3Root.buttonFont; font.bold: true }
                                                    MotionMouseArea { id: jnContinuousMA; anchors.fill: parent; enabled: robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart("j" + jn + "-"); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                    MotionMouseArea { id: jnStepMA; anchors.fill: parent; enabled: !robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onClicked: robotController.jogStep("j" + jn + "-", page3Root.stepValue) }
                                                }
                                                Rectangle {
                                                    width: parent.width - 120; height: 48; radius: 5; color: "transparent"; border.width: 1; border.color: root.cFunctionFieldBorder
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: root.cFunctionFieldStart }
                                                        GradientStop { position: 1.0; color: root.cFunctionFieldEnd }
                                                    }
                                                    Text { anchors.centerIn: parent; text: robotController.jointAngles.length > index ? robotController.jointAngles[index].toFixed(4) : "0.0000"; color: root.cWhiteText; font.pixelSize: page3Root.valueFont; font.bold: true }
                                                }
                                                Rectangle {
                                                    id: jpBtn
                                                    width: 58; height: 48; radius: 8
                                                    color: "transparent"
                                                    border.color: root.cServoRunBorder; border.width: 1
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: (jpContinuousMA.pressed || jpStepMA.pressed) ? root.pressGradientColor(root.cServoRunStart) : root.cServoRunStart }
                                                        GradientStop { position: 1.0; color: (jpContinuousMA.pressed || jpStepMA.pressed) ? root.pressGradientColor(root.cServoRunEnd) : root.cServoRunEnd }
                                                    }
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "+"; color: root.cWhiteText; font.pixelSize: page3Root.buttonFont; font.bold: true }
                                                    MotionMouseArea { id: jpContinuousMA; anchors.fill: parent; enabled: robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart("j" + jn + "+"); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                    MotionMouseArea { id: jpStepMA; anchors.fill: parent; enabled: !robotController.jogContinuous; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onClicked: robotController.jogStep("j" + jn + "+", page3Root.stepValue) }
                                                }
                                            }
                                        }
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }
                                        Row { spacing: 3; width: parent.width
                                            Repeater {
                                                id: jointInputs
                                                model: ["J1","J2","J3","J4","J5","J6"]
                                                delegate: Column { spacing: 2; width: (jointCol.width - 15) / 6
                                                    Text { text: modelData; color: root.cCardTitle; font.pixelSize: page3Root.helperFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                    Rectangle {
                                                        width: parent.width; height: 40; radius: 4; color: root.cDashCardField; border.color: root.cFunctionFieldBorder; border.width: 1
                                                        SmartTextInput {
                                                            focusHost: root
                                                            anchors { fill: parent; margins: 2 }
                                                            color: root.cWhiteText; font.pixelSize: page3Root.inputFont; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; clip: true
                                                            text: robotController.jointAngles.length > index ? robotController.jointAngles[index].toFixed(4) : "0"
                                                            selectByMouse: true; validator: DoubleValidator { notation: DoubleValidator.StandardNotation }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 8
                                                color: "transparent"; border.color: root.cGetButtonBorder
                                                border.width: 1
                                                gradient: Gradient {
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: gaMA.pressed ? root.pressGradientColor(root.cGetButton) : root.cGetButton }
                                                    GradientStop { position: 1.0; color: gaMA.pressed ? root.pressGradientColor(root.cGetButtonEnd) : root.cGetButtonEnd }
                                                }
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Row {
                                                    anchors.centerIn: parent
                                                    spacing: 6
                                                    Image {
                                                        source: "icons/hard_drive_download.svg"
                                                        width: 20; height: 20
                                                        sourceSize.width: 80
                                                        sourceSize.height: 80
                                                        fillMode: Image.PreserveAspectFit
                                                        smooth: true
                                                        mipmap: true
                                                        antialiasing: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                    Text {
                                                        text: "GET ANGLES"
                                                        color: root.cWhiteText
                                                        font.pixelSize: page3Root.labelFont
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                }
                                                MotionMouseArea { id: gaMA; anchors.fill: parent; hoverScale: 1.02; shadowEnabled: false; shimmerEnabled: false; onClicked: {
                                                    robotController.getAngles()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.jointAngles.length > i)
                                                            jointInputs.itemAt(i).children[1].children[0].text = robotController.jointAngles[i].toFixed(4)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 8
                                                color: "transparent"; border.color: root.cMovJButtonBorder
                                                border.width: 1
                                                gradient: Gradient {
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: mjMA.pressed ? root.pressGradientColor(root.cMovJButton) : root.cMovJButton }
                                                    GradientStop { position: 1.0; color: mjMA.pressed ? root.pressGradientColor(root.cMovJButtonEnd) : root.cMovJButtonEnd }
                                                }
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Row {
                                                    anchors.centerIn: parent
                                                    spacing: 6
                                                    Image {
                                                        source: "icons/send.svg"
                                                        width: 20; height: 20
                                                        sourceSize.width: 80
                                                        sourceSize.height: 80
                                                        fillMode: Image.PreserveAspectFit
                                                        smooth: true
                                                        mipmap: true
                                                        antialiasing: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                    Text {
                                                        text: "SEND MovJ"
                                                        color: root.cWhiteText
                                                        font.pixelSize: page3Root.labelFont
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                }
                                                MotionMouseArea { id: mjMA; anchors.fill: parent; hoverScale: 1.02; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: { var vals = []; for (var i = 0; i < 6; i++) vals.push(parseFloat(jointInputs.itemAt(i).children[1].children[0].text) || 0); robotController.startSendMoveJ(vals[0],vals[1],vals[2],vals[3],vals[4],vals[5]) } onReleased: robotController.stopSendMove(); onCanceled: robotController.stopSendMove() }
                                            }
                                        }

                                        // ── SAVE TO YAML ─────────────────────
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder; opacity: 0.5 }
                                        Text { text: "SAVE POSE TO YAML"; color: root.cCardTitle; font.pixelSize: page3Root.helperFont; font.bold: true; font.letterSpacing: 0.8 }

                                        Row { spacing: 4; width: parent.width
                                            // Name / comment input
                                            Rectangle {
                                                id: poseNameRect
                                                width: parent.width - 90 - 4; height: 42; radius: 6
                                                color: root.cDashCardField; border.color: root.cFunctionFieldBorder; border.width: 1
                                                // Placeholder hint
                                                Text {
                                                    anchors { fill: parent; leftMargin: 8; verticalCenter: parent.verticalCenter }
                                                    text: poseNameInput.text.length === 0 ? "---  Type pose name  ---" : ""
                                                    color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true
                                                    verticalAlignment: Text.AlignVCenter
                                                }
                                                SmartTextInput {
                                                    id: poseNameInput
                                                    focusHost: root
                                                    anchors { fill: parent; leftMargin: 8; rightMargin: 4; topMargin: 4; bottomMargin: 4 }
                                                    color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true
                                                    clip: true; selectByMouse: true; verticalAlignment: Text.AlignVCenter
                                                }
                                            }
                                            // SAVE button
                                            Rectangle {
                                                id: savePoseBtn
                                                width: 90; height: 42; radius: 6
                                                property bool saving: false
                                                color: "transparent"
                                                border.color: savePoseBtn.saving ? root.cAccent : root.cDashButtonBorder
                                                border.width: 1
                                                gradient: Gradient {
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: savePoseBtn.saving ? root.pressGradientColor(root.cDashButton) : (saveMA.pressed ? root.pressGradientColor(root.cDashButton) : root.cDashButton) }
                                                    GradientStop { position: 1.0; color: savePoseBtn.saving ? root.pressGradientColor(root.cDashButtonEnd) : (saveMA.pressed ? root.pressGradientColor(root.cDashButtonEnd) : root.cDashButtonEnd) }
                                                }
                                                Behavior on color { ColorAnimation { duration: 100 } }
                                                Row {
                                                    anchors.centerIn: parent
                                                    spacing: 5
                                                    visible: !savePoseBtn.saving

                                                    Image {
                                                        source: "icons/download.svg"
                                                        width: 20
                                                        height: 20
                                                        fillMode: Image.PreserveAspectFit
                                                        smooth: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                    Text {
                                                        text: "SAVE"
                                                        color: root.cWhiteText
                                                        font.pixelSize: page3Root.helperFont
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                }
                                                Text {
                                                    anchors.centerIn: parent
                                                    visible: savePoseBtn.saving
                                                    text: "✓ SAVED"
                                                    color: root.cWhiteText
                                                    font.pixelSize: page3Root.helperFont
                                                    font.bold: true
                                                }
                                                    MotionMouseArea {
                                                        id: saveMA; anchors.fill: parent
                                                        hoverScale: 1.02
                                                        pressScale: 0.976
                                                        shadowEnabled: false
                                                        shimmerEnabled: false
                                                    onClicked: {
                                                        var vals = []
                                                        for (var i = 0; i < 6; i++)
                                                            vals.push(parseFloat(jointInputs.itemAt(i).children[1].children[0].text) || 0)
                                                        robotController.saveJointPose(
                                                            poseNameInput.text,
                                                            vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]
                                                        )
                                                    }
                                                }
                                                Connections {
                                                    target: robotController
                                                    function onJointPoseSaved(success, message) {
                                                        savePoseBtn.saving = success
                                                        saveStatusText.text = message
                                                        saveStatusText.color = root.cWhiteText
                                                        saveStatusTimer.restart()
                                                    }
                                                }
                                                Timer {
                                                    id: saveStatusTimer; interval: 3000
                                                    onTriggered: { savePoseBtn.saving = false; saveStatusText.text = "" }
                                                }
                                            }
                                        }
                                        // Status toast
                                        Text {
                                            id: saveStatusText
                                            width: parent.width; wrapMode: Text.WordWrap
                                            text: ""; font.pixelSize: page3Root.helperFont
                                            color: root.cWhiteText
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder; opacity: 0.5 }
                                        Text { text: "LOAD SAVED POSE"; color: root.cCardTitle; font.pixelSize: page3Root.helperFont; font.bold: true; font.letterSpacing: 0.8 }

                                        Rectangle {
                                            id: savedPosesLoaderRect
                                            width: parent.width; height: 42; radius: 6
                                            color: "transparent"
                                            border.color: root.cDashButtonBorder; border.width: 1
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: loadMA.pressed ? root.pressGradientColor(root.cDashButton) : root.cDashButton }
                                                GradientStop { position: 1.0; color: loadMA.pressed ? root.pressGradientColor(root.cDashButtonEnd) : root.cDashButtonEnd }
                                            }

                                            property var savedPoses: []
                                            function refreshPoses() {
                                                savedPoses = robotController.getSavedPoses()
                                            }
                                            Component.onCompleted: refreshPoses()

                                            Row {
                                                anchors { left: parent.left; leftMargin: 8; verticalCenter: parent.verticalCenter }
                                                spacing: 7

                                                Image {
                                                    source: "icons/database_search.svg"
                                                    width: 20; height: 20
                                                    sourceSize.width: 80
                                                    sourceSize.height: 80
                                                    fillMode: Image.PreserveAspectFit
                                                    smooth: true
                                                    mipmap: true
                                                    antialiasing: true
                                                    anchors.verticalCenter: parent.verticalCenter
                                                }

                                                Text {
                                                    text: " LIST POSITION (" + savedPosesLoaderRect.savedPoses.length + ")"
                                                    color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true
                                                    anchors.verticalCenter: parent.verticalCenter
                                                }
                                            }

                                            Text {
                                                anchors { right: parent.right; rightMargin: 8; verticalCenter: parent.verticalCenter }
                                                text: "▼"
                                                color: root.cWhiteText; font.pixelSize: page3Root.helperFont
                                            }

                                            MotionMouseArea {
                                                id: loadMA
                                                anchors.fill: parent
                                                onClicked: {
                                                    savedPosesLoaderRect.refreshPoses()
                                                    poseSelectorPopup.open()
                                                }
                                            }
                                        }

                                    }
                                }

                                // ═══ IO + CONTROLS ═══
                                Rectangle {
                                    width: parent.width - (parent.width - parent.parent.colGap * 2) * 0.76 - parent.parent.colGap * 2; height: parent.height
                                    color: root.cControlPanel; border.color: root.cControlBorder; radius: 6
                                    GlassHighlight {}
                                    Column {
                                        id: ioCol
                                        anchors { fill: parent; margins: 8 }
                                        spacing: 6
                                        Row { spacing: 6
                                            Rectangle { width: 4; height: 16; radius: 1; color: root.cCardTitle; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "I/O CONTROL"; color: root.cCardTitle; font.pixelSize: page3Root.titleFont; font.bold: true; font.letterSpacing: 1.5 }
                                        }

                                        // Step Value
                                        Rectangle {
                                            id: continuousJogBtn
                                            width: parent.width; height: 34; radius: 5
                                            property bool selected: robotController.jogContinuous
                                            color: "transparent"
                                            border.color: selected ? root.cGetButtonBorder : root.cDashButtonBorder
                                            border.width: 1
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: continuousJogBtn.selected ? (continuousJogMA.pressed ? root.pressGradientColor(root.cGetButton) : root.cGetButton) : (continuousJogMA.pressed ? root.pressGradientColor(root.cDashButton) : root.cDashButton) }
                                                GradientStop { position: 1.0; color: continuousJogBtn.selected ? (continuousJogMA.pressed ? root.pressGradientColor(root.cGetButtonEnd) : root.cGetButtonEnd) : (continuousJogMA.pressed ? root.pressGradientColor(root.cDashButtonEnd) : root.cDashButtonEnd) }
                                            }
                                            Text { anchors.centerIn: parent; text: "CONTINUOUS JOG"; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                            MotionMouseArea { id: continuousJogMA; anchors.fill: parent; onClicked: robotController.setJogContinuous(true) }
                                        }
                                        Text { text: "STEP VALUE"; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                        Row { spacing: 4; width: parent.width
                                            Repeater {
                                                model: [0.1, 1, 5, 10]
                                                delegate: Rectangle {
                                                    id: stepBtn
                                                    required property var modelData
                                                    width: (ioCol.width - 12) / 4; height: 34; radius: 5
                                                    property bool selected: !robotController.jogContinuous && page3Root.stepValue === modelData
                                                    color: "transparent"
                                                    border.color: selected ? root.cGetButtonBorder : root.cDashButtonBorder
                                                    border.width: 1
                                                    gradient: Gradient {
                                                        orientation: Gradient.Horizontal
                                                        GradientStop { position: 0.0; color: stepBtn.selected ? (stepMA.pressed ? root.pressGradientColor(root.cGetButton) : root.cGetButton) : (stepMA.pressed ? root.pressGradientColor(root.cDashButton) : root.cDashButton) }
                                                        GradientStop { position: 1.0; color: stepBtn.selected ? (stepMA.pressed ? root.pressGradientColor(root.cGetButtonEnd) : root.cGetButtonEnd) : (stepMA.pressed ? root.pressGradientColor(root.cDashButtonEnd) : root.cDashButtonEnd) }
                                                    }
                                                    Text { anchors.centerIn: parent; text: modelData; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                                    MotionMouseArea { id: stepMA; anchors.fill: parent; onClicked: {
                                                        page3Root.stepValue = modelData
                                                        robotController.setJogContinuous(false)
                                                        robotController.setJogStepSize(modelData)
                                                    } }
                                                }
                                            }
                                        }

                                        // Hardware Speed (read-only from Dobot)
                                        Text { text: "HW SPEED"; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                        Rectangle {
                                            width: parent.width; height: 36; radius: 5
                                            color: "transparent"; border.color: root.cFunctionFieldBorder; border.width: 1
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: root.cFunctionFieldStart }
                                                GradientStop { position: 1.0; color: root.cFunctionFieldEnd }
                                            }
                                            Row {
                                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                                spacing: 6
                                                Text {
                                                    text: "⚙ Dobot:"
                                                    color: root.cWhiteText; font.pixelSize: page3Root.labelFont
                                                    anchors.verticalCenter: parent.verticalCenter
                                                }
                                                Text {
                                                    text: robotController.hwSpeedRatio + "%"
                                                    color: root.cWhiteText; font.pixelSize: page3Root.valueFont; font.bold: true
                                                    anchors.verticalCenter: parent.verticalCenter
                                                }
                                            }
                                        }

                                        // Set Speed (interactive slider)
                                        Text { text: "SET SPEED %"; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                        Row { spacing: 4; width: parent.width; height: 36
                                            Slider {
                                                id: speedSlider
                                                width: parent.width - 56; height: 34
                                                from: 1; to: 100; stepSize: 1; value: page3Root.speedVal
                                                onMoved: { page3Root.speedVal = Math.round(value) }
                                                onPressedChanged: { if (!pressed) robotController.setSpeedRatio(Math.round(value)) }
                                                background: Rectangle { x: speedSlider.leftPadding; y: speedSlider.topPadding + speedSlider.availableHeight / 2 - 4; width: speedSlider.availableWidth; height: 8; radius: 4; color: root.cDashButtonEnd; border.color: root.cDashButtonBorder
                                                    Rectangle { width: speedSlider.visualPosition * parent.width; height: parent.height; radius: 4; color: "#67d0ff" }
                                                }
                                                handle: Rectangle { x: speedSlider.leftPadding + speedSlider.visualPosition * (speedSlider.availableWidth - width); y: speedSlider.topPadding + speedSlider.availableHeight / 2 - 8; width: 18; height: 18; radius: 9; color: "#67d0ff"; border.color: "#fff" }
                                            }
                                            Rectangle {
                                                width: 50; height: 34; radius: 5; color: root.cFunctionFieldEnd; border.color: root.cFunctionFieldBorder; border.width: 1
                                                SmartTextInput { focusHost: root; anchors.centerIn: parent; width: 44; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true; horizontalAlignment: Text.AlignHCenter
                                                    text: page3Root.speedVal
                                                    validator: IntValidator { bottom: 1; top: 100 }
                                                    selectByMouse: true
                                                    onEditingFinished: { var v = Math.max(1, Math.min(100, parseInt(text) || 100)); page3Root.speedVal = v; speedSlider.value = v; robotController.setSpeedRatio(v) }
                                                }
                                            }
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                        // Gripper DO1 — valve 5/3: GRIPPING (ch0=T,ch1=F) / RELEASING (ch0=F,ch1=T)
                                        Text { text: "GRIPPER (DO1)"; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                        Row {
                                            id: rowGripper
                                            property bool isOn: robotController.gripperOn
                                            spacing: 6; width: parent.width
                                            IoToggleButton { width: (parent.width - 6) / 2; height: 42; lbl: "GRIPPING"; activeChoice: rowGripper.isOn; onClicked: robotController.setDigitalOutput(1, true) }
                                            IoToggleButton { width: (parent.width - 6) / 2; height: 42; lbl: "RELEASING"; activeChoice: !rowGripper.isOn; onClicked: robotController.setDigitalOutput(1, false) }
                                        }

                                        // Picker DO2 — valve 5/3: PICKING (ch2=T,ch3=F) / RELEASING (ch2=F,ch3=T)
                                        Text { text: "PICKER (DO2)"; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                        Row {
                                            id: rowPicker
                                            property bool isOn: robotController.pickerOn
                                            spacing: 6; width: parent.width
                                            IoToggleButton { width: (parent.width - 6) / 2; height: 42; lbl: "PICKING"; activeChoice: rowPicker.isOn; onClicked: robotController.setDigitalOutput(2, true) }
                                            IoToggleButton { width: (parent.width - 6) / 2; height: 42; lbl: "RELEASING"; activeChoice: !rowPicker.isOn; onClicked: robotController.setDigitalOutput(2, false) }
                                        }

                                        // Cyl loadcell DO6 — sau khi đổi dây khí: true = RELEASING/mở (nhả), false = GRIPPING/kẹp
                                        Text { text: "CYL LOADCELL (DO6)"; color: root.cWhiteText; font.pixelSize: page3Root.labelFont; font.bold: true }
                                        Row {
                                            id: rowCylLoadcell
                                            property bool isOn: robotController.cylLoadcellOn
                                            spacing: 6; width: parent.width
                                            IoToggleButton { width: (parent.width - 6) / 2; height: 42; lbl: "GRIPPING"; activeChoice: !rowCylLoadcell.isOn; onClicked: robotController.setDigitalOutput(6, false) }
                                            IoToggleButton { width: (parent.width - 6) / 2; height: 42; lbl: "RELEASING"; activeChoice: rowCylLoadcell.isOn; onClicked: robotController.setDigitalOutput(6, true) }
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                        // System actions — tỉ lệ + bo góc đồng bộ CameraPage SYSTEM CONTROL:
                                        // ENABLE trên cùng, STOP + CLEAR ERROR cạnh nhau, EMERGENCY dưới cùng.
                                        CBtn {
                                            lbl: "ENABLE"
                                            w: parent.width; h: 52
                                            fontSize: 15
                                            bg: root.cBtnPrimaryStart; bgEnd: root.cBtnPrimaryEnd; bc: root.cBtnPrimaryBorder; tc: "#ffffff"
                                            onClicked: robotController.enableSystem(true)
                                        }

                                        Row {
                                            spacing: 6; width: parent.width
                                            CBtn {
                                                lbl: "STOP"
                                                w: (parent.width - 6) / 2; h: 52
                                                fontSize: 15
                                                bg: root.cBtnDangerStart; bgEnd: root.cBtnDangerEnd; bc: root.cBtnDangerBorder; tc: "#ffffff"
                                                onClicked: mainWindow.stopSynchronizedSystems()
                                            }
                                            CBtn {
                                                lbl: "CLEAR ERROR"
                                                w: (parent.width - 6) / 2; h: 52
                                                fontSize: 13
                                                bg: root.cBtnClearStart; bgEnd: root.cBtnClearEnd; bc: root.cBtnActionBorder; tc: root.cBtnClearText
                                                onClicked: robotController.clearError()
                                            }
                                        }

                                    }
                                }
                            }
                        } // manual mode

                        // ════════════ AUTOMATED MODE OVERLAY ════════════
                        Item {
                            visible: !page3Root.manualEnabled
                            anchors.fill: parent; z: 100

                            // Dim background & block clicks to manual elements
                            Rectangle {
                                anchors.fill: parent; color: "#000000"; opacity: 0.65; radius: 6
                                MotionMouseArea {
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    motionEnabled: false
                                    shadowEnabled: false
                                    shimmerEnabled: false
                                }
                            }

                            Column {
                                anchors.centerIn: parent; spacing: 14
                                
                                Text {
                                    text: "🔒 MANUAL CONTROL LOCKED\nRobot đang chạy — không thể JOG"
                                    color: root.cWhiteText; font.pixelSize: 16; font.bold: true
                                    horizontalAlignment: Text.AlignHCenter; lineHeight: 1.5
                                    anchors.horizontalCenter: parent.horizontalCenter
                                }

                                Row {
                                    spacing: 8
                                    anchors.horizontalCenter: parent.horizontalCenter

                                    CBtn {
                                        lbl: "STOP"
                                        w: 150; h: 48
                                        fontSize: 15
                                        bg: root.cBtnDangerStart; bgEnd: root.cBtnDangerEnd; bc: root.cBtnDangerBorder; tc: "#ffffff"
                                        onClicked: mainWindow.stopSynchronizedSystems()
                                    }

                                    CBtn {
                                        lbl: "ENABLE"
                                        w: 150; h: 48
                                        fontSize: 15
                                        bg: root.cBtnPrimaryStart; bgEnd: root.cBtnPrimaryEnd; bc: root.cBtnPrimaryBorder; tc: "#ffffff"
                                        onClicked: robotController.enableSystem(true)
                                    }

                                    CBtn {
                                        lbl: "CLEAR ERROR"
                                        w: 170; h: 48
                                        fontSize: 13
                                        bg: root.cBtnClearStart; bgEnd: root.cBtnClearEnd; bc: root.cBtnActionBorder; tc: root.cBtnClearText
                                        onClicked: robotController.clearError()
                                    }
                                }
                            }
                        }
                    } // contentArea

                    // ════════════════ ROBOT LOG BAR ══════════════════════
                    Rectangle {
                        id: robotLogBar
                        anchors { left: parent.left; right: parent.right; bottom: parent.bottom }
                        height: 160; radius: 6; color: root.cControlPanel; border.color: root.cControlBorder
                        GlassHighlight {}
                        Column {
                            anchors { fill: parent; margins: 8 }
                            spacing: 4
                            RowLayout {
                                width: parent.width; height: 18
                                Text { text: "ROBOT LOG"; color: root.cCardTitle; font.pixelSize: page3Root.titleFont; font.bold: true; font.letterSpacing: 1.5 }
                                Item { Layout.fillWidth: true }
                                CBtn { lbl:"Clear"; Layout.preferredWidth: implicitWidth; Layout.preferredHeight: implicitHeight; padV:3; padH:8; fontSize:14; bg:root.cBtnClearStart; bgEnd:root.cBtnClearEnd; bc:root.cBtnActionBorder; tc:root.cBtnClearText; onClicked:robotController.clearLog() }
                            }
                            Rectangle {
                                width: parent.width; height: parent.height - 22
                                color: "#06101d"; border.color: root.cBorder; radius: 4
                                ListView {
                                    anchors { fill: parent; margins: 6 }
                                    model: robotController.logEntries
                                    clip: true; spacing: 2
                                    verticalLayoutDirection: ListView.BottomToTop
                                    delegate: Text {
                                        width: parent ? parent.width : 100
                                        text: "[" + modelData.time + "] " + modelData.msg
                                        font.pixelSize: page3Root.helperFont
                                        color: root.cWhiteText
                                        wrapMode: Text.WordWrap
                                    }
                                }
                            }
                        }
                    }

                } // p3Inner

                // ── POPUP: SELECT SAVED POSE ──
                Rectangle {
                    id: poseSelectorPopup
                    anchors.fill: parent
                    color: "#cc000000" // dim background
                    visible: false
                    z: 9999 // ensure it is on top of everything

                    // Close on clicking background
                    MotionMouseArea {
                        anchors.fill: parent
                        motionEnabled: false
                        shadowEnabled: false
                        shimmerEnabled: false
                        onClicked: poseSelectorPopup.visible = false
                    }

                    Rectangle {
                        id: popupBg
                        width: parent.width * 0.9; height: parent.height * 0.8
                        anchors.centerIn: parent
                        color: "#04080f"
                        border.color: root.cFunctionFieldBorder; border.width: 1
                        radius: 8

                        // Prevent clicking inside from closing
                        MotionMouseArea {
                            anchors.fill: parent
                            preventStealing: true
                            motionEnabled: false
                            shadowEnabled: false
                            shimmerEnabled: false
                        }

                        Column {
                            anchors { fill: parent; margins: 14 }
                            spacing: 10

                            Item {
                                width: parent.width; height: 34
                                Text {
                                    text: "📋 CHỌN TOẠ ĐỘ ROBOT ĐÃ LƯU"
                                    color: root.cWhiteText; font.pixelSize: page3Root.titleFont; font.bold: true; font.letterSpacing: 1.5
                                    anchors.verticalCenter: parent.verticalCenter
                                    anchors.left: parent.left
                                }
                                Rectangle {
                                    width: 34; height: 34; radius: 17
                                    color: closeMA.pressed ? root.pressColor("#4a1e1c") : "#1c0f0e"
                                    border.color: "#f0735c"
                                    anchors.right: parent.right
                                    anchors.verticalCenter: parent.verticalCenter
                                    Text { anchors.centerIn: parent; text: "✕"; color: root.cWhiteText; font.pixelSize: page3Root.titleFont; font.bold: true }
                                    MotionMouseArea {
                                        id: closeMA; anchors.fill: parent; onClicked: poseSelectorPopup.visible = false
                                    }
                                }
                            }

                            Rectangle { width: parent.width; height: 1; color: "#14263c" }

                            Item {
                                width: parent.width; height: popupBg.height - 83
                                
                                ListView {
                                    id: poseListView
                                    anchors.fill: parent
                                    clip: true
                                    spacing: 8
                                    model: []

                                    delegate: Rectangle {
                                        width: poseListView.width; height: 72
                                        color: itemMA.pressed ? root.pressColor("#0c1726") : (itemMA.containsMouse ? "#081627" : "#06101d")
                                        border.color: itemMA.containsMouse ? root.cFunctionFieldBorder : "#0c1726"; border.width: 1
                                        radius: 6

                                        Row {
                                            anchors { fill: parent; leftMargin: 12; rightMargin: 12 }
                                            spacing: 16

                                            // Left side: Comment & Grid coordinates
                                            Column {
                                                anchors.verticalCenter: parent.verticalCenter
                                                width: parent.width - 104
                                                spacing: 4
                                                
                                                Text {
                                                    text: modelData.name ? modelData.name : "Không có tên/ghi chú"
                                                    color: root.cWhiteText; font.pixelSize: 22; font.bold: true
                                                    elide: Text.ElideRight
                                                }
                                                
                                                // Grid-like layout for coordinates with vertical alignment and clear margins
                                                Row {
                                                    spacing: 8
                                                    width: parent.width
                                                    
                                                    Repeater {
                                                        model: [
                                                            { label: "J1", val: modelData.j1 },
                                                            { label: "J2", val: modelData.j2 },
                                                            { label: "J3", val: modelData.j3 },
                                                            { label: "J4", val: modelData.j4 },
                                                            { label: "J5", val: modelData.j5 },
                                                            { label: "J6", val: modelData.j6 }
                                                        ]
                                                        
                                                        delegate: Row {
                                                            spacing: 4
                                                            Text {
                                                                text: model.modelData.label + ":"
                                                                color: root.cWhiteText
                                                                font { pixelSize: 18; bold: true }
                                                                anchors.verticalCenter: parent.verticalCenter
                                                            }
                                                            Text {
                                                                text: model.modelData.val.toFixed(2)
                                                                color: root.cWhiteText
                                                                font { pixelSize: 19; family: "monospace"; bold: true }
                                                                anchors.verticalCenter: parent.verticalCenter
                                                            }
                                                        }
                                                    }
                                                }
                                            }

                                            // Apply button
                                            Rectangle {
                                                width: 88; height: 42; radius: 6
                                                anchors.verticalCenter: parent.verticalCenter
                                                color: applyMA.pressed ? root.pressColor("#1f9e86") : "#0a2418"
                                                border.color: "#1f9e86"
                                                
                                                Text {
                                                    anchors.centerIn: parent
                                                    text: "APPLY"
                                                    color: root.cWhiteText
                                                    font.pixelSize: 16; font.bold: true; font.letterSpacing: 1
                                                }
                                                
                                                MotionMouseArea {
                                                    id: applyMA; anchors.fill: parent
                                                    onClicked: {
                                                        jointInputs.itemAt(0).children[1].children[0].text = modelData.j1.toFixed(4)
                                                        jointInputs.itemAt(1).children[1].children[0].text = modelData.j2.toFixed(4)
                                                        jointInputs.itemAt(2).children[1].children[0].text = modelData.j3.toFixed(4)
                                                        jointInputs.itemAt(3).children[1].children[0].text = modelData.j4.toFixed(4)
                                                        jointInputs.itemAt(4).children[1].children[0].text = modelData.j5.toFixed(4)
                                                        jointInputs.itemAt(5).children[1].children[0].text = modelData.j6.toFixed(4)
                                                        poseSelectorPopup.visible = false
                                                    }
                                                }
                                            }
                                        }

                                        MotionMouseArea {
                                            id: itemMA
                                            anchors.fill: parent
                                            hoverEnabled: true
                                            onClicked: {
                                                jointInputs.itemAt(0).children[1].children[0].text = modelData.j1.toFixed(4)
                                                jointInputs.itemAt(1).children[1].children[0].text = modelData.j2.toFixed(4)
                                                jointInputs.itemAt(2).children[1].children[0].text = modelData.j3.toFixed(4)
                                                jointInputs.itemAt(3).children[1].children[0].text = modelData.j4.toFixed(4)
                                                jointInputs.itemAt(4).children[1].children[0].text = modelData.j5.toFixed(4)
                                                jointInputs.itemAt(5).children[1].children[0].text = modelData.j6.toFixed(4)
                                                poseSelectorPopup.visible = false
                                            }
                                        }
                                    }
                                }
                                
                                // Custom Scrollbar
                                Rectangle {
                                    id: scrollbar
                                    anchors { right: parent.right; top: parent.top; bottom: parent.bottom; rightMargin: 2 }
                                    width: 4; radius: 2; color: "#0c1726"
                                    visible: poseListView.height < poseListView.contentHeight
                                    
                                    Rectangle {
                                        width: parent.width; radius: 2; color: "#67d0ff"
                                        height: Math.max(20, poseListView.height * (poseListView.height / poseListView.contentHeight))
                                        y: poseListView.visibleArea.yPosition * poseListView.height
                                    }
                                }
                            }
                        }
                    }

                    function open() {
                        poseListView.model = robotController.getSavedPoses()
                        visible = true
                    }
                }
            } // Page 3

            // ── PAGE 4: FILL HP CONTROL (redesigned — see FillHpTab.qml) ──
            FillHpTab { focusHost: root }

            // ── PAGE 5: INK SYSTEM ──────────────────────────────────────
            InkTab { focusHost: root }

            // ── PAGE 6: PRODUCTION OUTPUT ───────────────────────────────
            ProductionTab { focusHost: root }

        } // StackLayout

        Item {
            id: contentReadOnlyShield
            anchors { top: stack.top; left: stack.left; right: stack.right; bottom: stack.bottom }
            visible: root.contentReadOnly
            z: 900

            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.AllButtons
                hoverEnabled: true
                preventStealing: true
                onClicked: mouse.accepted = true
                onPressed: mouse.accepted = true
                onReleased: mouse.accepted = true
                onWheel: wheel.accepted = true
            }

            Rectangle {
                anchors.top: parent.top
                anchors.right: parent.right
                anchors.topMargin: 14
                anchors.rightMargin: 14
                width: 250
                height: 48
                radius: 8
                color: "#cc06101d"
                border.color: root.operatorTechnicalLocked ? root.cOrange : root.cGreen
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 0

                    Text {
                        Layout.fillWidth: true
                        text: root.operatorTechnicalLocked ? "TECHNICAL VIEW ONLY" : "VIEWER MODE"
                        color: root.operatorTechnicalLocked ? root.cOrange : root.cAccent
                        font.pixelSize: 14
                        font.bold: true
                        elide: Text.ElideRight
                    }

                    Text {
                        Layout.fillWidth: true
                        text: "Viewing enabled, controls locked"
                        color: root.cText
                        font.pixelSize: 11
                        elide: Text.ElideRight
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // REUSABLE: FunctionLabel — section labels inside servo cards
        // ════════════════════════════════════════════════════════════
        component FunctionLabel: Item {
            id: functionLabel

            property string lbl: ""
            property int fontSize: 14

            Text {
                anchors.centerIn: parent
                text: functionLabel.lbl
                color: root.cFunctionLabelText
                font.pixelSize: functionLabel.fontSize
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        // ════════════════════════════════════════════════════════════
        // REUSABLE: CBtn — matches HTML .btn
        // ════════════════════════════════════════════════════════════
        component CBtn: Rectangle {
            id: cbr
            property string lbl: ""
            property string iconSource: ""
            property color bg:    root.cCard
            property color bgEnd: bg       // when bgEnd != bg, use horizontal gradient
            property color selectedBg: bg
            property color selectedBgEnd: bgEnd
            property color selectedTc: tc
            property color bc:    root.cBorder
            property color tc:    root.cText
            property bool  glassStyle: true
            property color visualBg: isSelected ? selectedBg : bg
            property color visualBgEnd: isSelected ? selectedBgEnd : bgEnd
            property color renderedBg: (glassStyle && visualBg.a > 0) ? Qt.rgba(visualBg.r, visualBg.g, visualBg.b, 0.96) : visualBg
            property color renderedBgEnd: (glassStyle && visualBgEnd.a > 0) ? Qt.rgba(visualBgEnd.r, visualBgEnd.g, visualBgEnd.b, 0.96) : visualBgEnd
            property color renderedBc: glassStyle ? Qt.rgba(bc.r, bc.g, bc.b, 0.90) : bc
            readonly property bool gradientFilled: renderedBgEnd !== renderedBg && renderedBg.a > 0 && renderedBgEnd.a > 0
            readonly property color blendedBorder: renderedBc
            property bool  active: true
            property bool  clickEnabled: active
            property real  inactiveOpacity: 0.4
            property int   padV: 6
            property int   padH: 12
            property int   fontSize: 16
            property int   labelHorizontalAlignment: lbl.indexOf("CYL") === 0
                                                     ? Text.AlignHCenter
                                                     : Text.AlignLeft
            readonly property int displayFontSize: fontSize
            property int   w: 0
            property int   h: 0
            property bool  _pressed: false
            property bool  _hovered: false
            property bool  isSelected: false
            property bool  selectedInset: true
            // Khi blinking=true: viền + nền nhấp nháy thu hút sự chú ý (vd hint từ
            // node Python qua uiHint). Auto-stop khi blinking=false. Animation 600ms/chu kỳ.
            property bool  blinking: false

            signal clicked(); signal pressed(); signal released()

            implicitWidth:  w > 0 ? w : cbrT.implicitWidth + padH * 2
                            + (iconSource !== "" ? displayFontSize * 1.45 + 7 : 0)
            implicitHeight: h > 0 ? h : cbrT.implicitHeight + padV * 2
            radius: glassStyle ? 10 : 6
            property bool _heldVisual: _pressed

            // Outline (renderedBg.a==0): pressed/selected/hover hiện fill teal mờ
            // (glass) lấy từ màu viền. Filled (a>0): darker/lighter như cũ.
            color: renderedBgEnd !== renderedBg ? "transparent" : (
                !active ? renderedBg :
                _pressed ? (renderedBg.a > 0 ? Qt.darker(renderedBg, root.cPressDarken) : Qt.rgba(renderedBc.r, renderedBc.g, renderedBc.b, 0.28)) :
                isSelected ? (renderedBg.a > 0 ? Qt.lighter(renderedBg, 1.08) : Qt.rgba(renderedBc.r, renderedBc.g, renderedBc.b, 0.20)) :
                _hovered ? (renderedBg.a > 0 ? Qt.lighter(renderedBg, 1.06) : Qt.rgba(renderedBc.r, renderedBc.g, renderedBc.b, 0.10)) :
                renderedBg
            )
            border.color: {
                if (isSelected) return Qt.lighter(blendedBorder, gradientFilled ? 1.06 : 1.12)
                if (_hovered) return Qt.lighter(blendedBorder, gradientFilled ? 1.06 : 1.08)
                return blendedBorder
            }
            border.width: 1
            opacity: active ? 1.0 : inactiveOpacity

            Behavior on color        { ColorAnimation { duration: 100 } }
            Behavior on border.color { ColorAnimation { duration: 100 } }
            Behavior on opacity      { NumberAnimation { duration: 150 } }

            // Gradient background (only active when bgEnd != bg)
            Rectangle {
                anchors.fill: parent
                radius: parent.radius
                visible: cbr.renderedBgEnd !== cbr.renderedBg
                z: -1
                gradient: Gradient {
                    orientation: Gradient.Horizontal
                    GradientStop { position: 0.0; color: cbr._heldVisual ? Qt.darker(cbr.renderedBg, root.cPressGradientDarken) : cbr.renderedBg }
                    GradientStop { position: 1.0; color: cbr._heldVisual ? Qt.darker(cbr.renderedBgEnd, root.cPressGradientDarken) : cbr.renderedBgEnd }
                }
            }

            Rectangle {
                anchors.fill: parent
                radius: parent.radius
                visible: cbr.glassStyle && cbr.renderedBg.a > 0
                z: -0.4
                gradient: Gradient {
                    orientation: Gradient.Vertical
                    GradientStop { position: 0.0; color: cbr._heldVisual ? Qt.rgba(0, 0, 0, 0.12) : Qt.rgba(1, 1, 1, 0.18) }
                    GradientStop { position: 0.52; color: cbr._heldVisual ? Qt.rgba(0, 0, 0, 0.08) : Qt.rgba(1, 1, 1, 0.06) }
                    GradientStop { position: 1.0; color: cbr._heldVisual ? Qt.rgba(0, 0, 0, 0.22) : Qt.rgba(0, 0, 0, 0.08) }
                }
            }

            // Glow effect when pressed
            Rectangle {
                anchors.fill: parent; anchors.margins: -2
                radius: parent.radius + 2; color: "transparent"
                border.color: "transparent"
                border.width: 3; z: -1
                Behavior on border.color { ColorAnimation { duration: 100 } }
            }

            Rectangle {
                anchors.fill: parent
                radius: parent.radius
                color: "#000000"
                opacity: cbr._pressed ? 0.26 : (cbr.isSelected ? 0.18 : 0.0)
                visible: opacity > 0
                Behavior on opacity { NumberAnimation { duration: 90 } }
            }

            // Blink overlay khi blinking=true (UI hint từ node Python)
            Rectangle {
                id: blinkOverlay
                anchors.fill: parent; anchors.margins: -3
                radius: parent.radius + 3
                color: "transparent"
                border.color: root.cAccent
                border.width: 4
                opacity: cbr.blinking && !root.continuousAnimationsEnabled ? 1 : 0
                visible: cbr.blinking
                z: 2
                SequentialAnimation on opacity {
                    loops: Animation.Infinite
                    running: root.continuousAnimationsEnabled && cbr.blinking
                    NumberAnimation { to: 1.0; duration: 350; easing.type: Easing.InOutQuad }
                    NumberAnimation { to: 0.2; duration: 350; easing.type: Easing.InOutQuad }
                }
            }

            // Inner shadow to simulate physically sunken pressed state
            Rectangle {
                anchors.fill: parent
                radius: parent.radius
                color: "transparent"
                border.color: "black"
                border.width: cbr.isSelected ? 4 : 0
                opacity: 0.6
                visible: cbr.isSelected && cbr.selectedInset
            }

            Row {
                anchors.centerIn: parent
                anchors.verticalCenterOffset: (cbr.isSelected || cbr._pressed) ? 2 : 0
                spacing: cbr.iconSource !== "" ? 7 : 0

                Item {
                    width: cbr.iconSource !== "" ? cbr.displayFontSize * 1.45 : 0
                    height: cbr.displayFontSize * 1.45
                    visible: cbr.iconSource !== ""

                    Image {
                        id: cbrImg
                        anchors.fill: parent
                        source: cbr.iconSource
                        fillMode: Image.PreserveAspectFit
                        smooth: true
                        visible: false
                    }

                    ColorOverlay {
                        anchors.fill: cbrImg
                        source: cbrImg
                        color: root.cWhiteText
                    }
                }

                Text {
                    id: cbrT
                    text: cbr.lbl
                    color: cbr.isSelected ? cbr.selectedTc : cbr.tc
                    font.pixelSize: cbr.displayFontSize
                    font.bold: true
                    horizontalAlignment: cbr.labelHorizontalAlignment
                    anchors.verticalCenter: parent.verticalCenter
                    Behavior on color { ColorAnimation { duration: 80 } }
                }
            }

            MotionMouseArea { anchors.fill: parent; hoverEnabled: true
                enabled: cbr.clickEnabled
                hoverScale: cbr.glassStyle ? 1.012 : 1.012
                pressScale: cbr.glassStyle ? 0.985 : 0.99
                shadowEnabled: false
                shimmerEnabled: cbr.glassStyle && cbr.active
                shimmerWhilePressed: true
                shimmerColor: cbr.glassStyle ? "#88ffffff" : "#55d4faff"
                raiseOnHover: true
                onClicked:       { if(cbr.clickEnabled) cbr.clicked() }
                onPressed:       {
                    root.dismissDataInput()
                    cbr._pressed = true
                    cbr.pressed()
                }
                onReleased:      { cbr._pressed = false; cbr.released() }
                onEntered:       cbr._hovered = true
                onExited:        { cbr._hovered = false; cbr._pressed = false }
                onCanceled:      { cbr._hovered = false; cbr._pressed = false }
            }
        }

        component IoToggleButton: Rectangle {
            id: ioBtn
            property string lbl: ""
            property bool activeChoice: false
            property bool pressed: false
            property bool hovered: false
            readonly property color gradStart: activeChoice ? root.cGetButton : root.cMovJButton
            readonly property color gradEnd: activeChoice ? root.cGetButtonEnd : root.cMovJButtonEnd
            readonly property color gradBorder: activeChoice ? root.cGetButtonBorder : root.cMovJButtonBorder

            signal clicked()

            radius: 8
            color: "transparent"
            border.color: gradBorder
            border.width: 1
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.0; color: ioBtn.pressed ? root.pressGradientColor(ioBtn.gradStart) : ioBtn.gradStart }
                GradientStop { position: 1.0; color: ioBtn.pressed ? root.pressGradientColor(ioBtn.gradEnd) : ioBtn.gradEnd }
            }

            Text {
                anchors.centerIn: parent
                anchors.verticalCenterOffset: ioBtn.pressed ? 1 : 0
                text: ioBtn.lbl
                color: root.cWhiteText
                font.pixelSize: page3Root.labelFont
                font.bold: true
            }

            MotionMouseArea {
                anchors.fill: parent
                hoverEnabled: true
                hoverScale: 1.02
                pressScale: 0.985
                shadowEnabled: false
                shimmerEnabled: false
                raiseOnHover: false
                onClicked: ioBtn.clicked()
                onPressed: ioBtn.pressed = true
                onReleased: ioBtn.pressed = false
                onEntered: ioBtn.hovered = true
                onExited: { ioBtn.hovered = false; ioBtn.pressed = false }
                onCanceled: { ioBtn.hovered = false; ioBtn.pressed = false }
            }
        }

        // ════════════════════════════════════════════════════════════
        // REUSABLE: ConfigCard — compact row position table
        // ════════════════════════════════════════════════════════════
        component ConfigCard: Rectangle {
            id: cfgCard
            property string title: ""
            property string configKey: ""
            property var configSource: ({})

            // height fill từ Layout parent
            color: root.cBg2; border.color: root.cBorder; radius: 6
            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

            Column {
                id: cfgCol
                anchors { fill: parent; margins: 8 }
                spacing: 2

                Text { text: cfgCard.title; color: root.cCardTitle; font.pixelSize: 15; font.bold: true; font.letterSpacing: 1.5 }

                Row { width: parent.width
                    Repeater { model: ["Row","Position (mm)","Mô tả"]
                        delegate: Text { text: modelData; color: root.cWhiteText; font.pixelSize: 11; font.bold: true
                            width: index===0?46:index===1?94:parent.width-140
                            font.capitalization: Font.AllUppercase; font.letterSpacing: 1 } }
                }
                Rectangle { width: parent.width; height: 1; color: root.cBorder }

                Repeater {
                    id: cfgRepeater
                    model: [10,9,8,7,6,5,4,3,2,1]
                    delegate: Rectangle {
                        required property int modelData
                        required property int index
                        width: cfgCol.width; height: 38
                        color: index % 2 === 0 ? "transparent" : "#06101d"
                        property alias inputText: rowInput.text
                        property int rowNum: modelData

                        Row {
                            anchors.verticalCenter: parent.verticalCenter; spacing: 0
                            Text { text: "R"+modelData; color: root.cWhiteText; font.pixelSize: 13; font.bold: true
                                   width: 46; anchors.verticalCenter: parent.verticalCenter }
                            Rectangle { width: 94; height: 30; radius: 4; color: "#081627"; border.color: root.cFieldBorder; border.width: 1
                                SmartTextInput { id: rowInput; focusHost: root; anchors { fill: parent; margins: 4 }
                                    text: "0.0"
                                    font.pixelSize: 14; font.family: "monospace"; color: root.cWhiteText
                                    horizontalAlignment: TextInput.AlignHCenter
                                    validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 2 }
	                                    Connections {
	                                        function onConfigRevisionChanged() {
	                                            var tbl = page2Root.parsedConfig[cfgCard.configKey]
	                                            if (tbl && tbl[String(modelData)] !== undefined)
	                                                rowInput.text = String(tbl[String(modelData)])
	                                            else
	                                                rowInput.text = ""
	                                        }
	                                        target: page2Root
	                                    }
                                }
                            }
                            Item { width: 4 }
                            Text {
                                text: modelData===10?"Top":modelData===1?"Bot":""
                                color: root.cWhiteText; font.pixelSize: 12; anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                        Rectangle { width: parent.width; height: 1; color: "#0c1726"; anchors.bottom: parent.bottom }
                    }
                }

                Row { spacing: 6; topPadding: 8
                    CBtn { lbl:"Save"; iconSource:"icons/download.svg"; padV:8; padH:18; fontSize: 17; bg:root.cBtnPrimaryStart; bgEnd:root.cBtnPrimaryEnd; bc:root.cBtnPrimaryBorder; tc:"#ffffff"
                        onClicked: {
                            var positions = {}
                            for (var i = 0; i < cfgRepeater.count; i++) {
                                var item = cfgRepeater.itemAt(i)
	                                if (item && item.inputText !== "") positions[String(item.rowNum)] = parseFloat(item.inputText) || 0.0
	                            }
	                            cartridgeController.saveConfig(cfgCard.configKey, JSON.stringify(positions))
                        }
                    }
	                    CBtn { lbl:"↺ Reset"; padV:8; padH:14; fontSize: 17; bg:root.cBtnBaseStart; bgEnd:root.cBtnBaseEnd; bc:root.cBtnBaseBorder; tc:root.cBtnBaseText
	                        onClicked: cartridgeController.getConfig()
	                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // REUSABLE: ConfigZoneCard — compact min/max/target table
        // ════════════════════════════════════════════════════════════
        component ConfigZoneCard: Rectangle {
            id: cfgZoneCard
            property string title: ""
            property string configKey: ""
            property var configSource: ({})

            color: root.cBg2; border.color: root.cBorder; radius: 6
            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

            Flickable {
                id: cfgZoneFlick
                anchors { fill: parent; margins: 8 }
                contentWidth: width; contentHeight: cfgZoneCol.height + 20
                clip: true

                Column {
                    id: cfgZoneCol
                    width: parent.width
                    spacing: 6

                    Text { text: cfgZoneCard.title; color: root.cCardTitle; font.pixelSize: 18; font.bold: true; font.letterSpacing: 1.5 }

                    Row { width: parent.width; height: 24; spacing: parent.width * 0.02
                        Text { text: "Row"; color: root.cWhiteText; font.pixelSize: 14; font.bold: true; width: parent.width * 0.12; height: parent.height; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.capitalization: Font.AllUppercase }
                        Text { text: "Max"; color: root.cWhiteText; font.pixelSize: 14; font.bold: true; width: parent.width * 0.23; height: parent.height; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.capitalization: Font.AllUppercase }
                        Text { text: "Min"; color: root.cWhiteText; font.pixelSize: 14; font.bold: true; width: parent.width * 0.23; height: parent.height; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.capitalization: Font.AllUppercase }
                        Text { text: "Target"; color: root.cWhiteText; font.pixelSize: 14; font.bold: true; width: parent.width * 0.23; height: parent.height; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.capitalization: Font.AllUppercase }
                        Text { text: "Loc"; color: root.cWhiteText; font.pixelSize: 14; font.bold: true; width: parent.width * 0.10; height: parent.height; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.capitalization: Font.AllUppercase }
                    }
                    Rectangle { width: parent.width; height: 1; color: root.cBorder }

                    Repeater {
                        id: cfgZoneRepeater
                        model: [10,9,8,7,6,5,4,3,2,1]
                        delegate: Rectangle {
                            required property int modelData
                            required property int index
                            width: cfgZoneCol.width; height: 46
                            color: index % 2 === 0 ? "transparent" : "#06101d"
                            property alias minText: minInp.text
                            property alias maxText: maxInp.text
                            property alias tgtText: tgtInp.text
                            property int rowNum: modelData

                            Row {
                                anchors.verticalCenter: parent.verticalCenter
                                width: parent.width
                                spacing: parent.width * 0.02
                                Text { text: "R"+modelData; color: root.cWhiteText; font.pixelSize: 18; font.bold: true; width: parent.width * 0.12; height: parent.height; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; anchors.verticalCenter: parent.verticalCenter }

	                                Rectangle { width: parent.width * 0.23; height: 36; radius: 5; color: "#081627"; border.color: root.cFieldBorder; border.width: 1
	                                    SmartTextInput { id: minInp; focusHost: root; anchors { fill: parent; margins: 3 } text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true; color: root.cWhiteText; horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
	                                        Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; minInp.text = (tbl && tbl[String(modelData)] !== undefined) ? String(tbl[String(modelData)][0]) : "" } } } }
	                                Rectangle { width: parent.width * 0.23; height: 36; radius: 5; color: "#081627"; border.color: root.cFieldBorder; border.width: 1
	                                    SmartTextInput { id: maxInp; focusHost: root; anchors { fill: parent; margins: 3 } text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true; color: root.cWhiteText; horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
	                                        Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; maxInp.text = (tbl && tbl[String(modelData)] !== undefined) ? String(tbl[String(modelData)][1]) : "" } } } }
	                                Rectangle { width: parent.width * 0.23; height: 36; radius: 5; color: "#081627"; border.color: root.cFieldBorder; border.width: 1
	                                    SmartTextInput { id: tgtInp; focusHost: root; anchors { fill: parent; margins: 3 } text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true; color: root.cWhiteText; horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
	                                        Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; tgtInp.text = (tbl && tbl[String(modelData)] !== undefined) ? String(tbl[String(modelData)][2]) : "" } } } }

                                Text { text: modelData===10?"Top":modelData===1?"Bot":""; color: root.cWhiteText; font.pixelSize: 14; font.bold: true; width: parent.width * 0.10; height: parent.height; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; anchors.verticalCenter: parent.verticalCenter }
                            }
                            Rectangle { width: parent.width; height: 1; color: "#0c1726"; anchors.bottom: parent.bottom }
                        }
                    }

                    Row { spacing: 8; topPadding: 8
                        CBtn { lbl:"Save"; iconSource:"icons/download.svg"; padV:10; padH:22; fontSize: 18; bg:root.cBtnPrimaryStart; bgEnd:root.cBtnPrimaryEnd; bc:root.cBtnPrimaryBorder; tc:"#ffffff"
                            onClicked: {
                                var positions = {}
                                for (var i = 0; i < cfgZoneRepeater.count; i++) {
                                    var item = cfgZoneRepeater.itemAt(i)
                                    if (item) {
	                                      if (item.minText !== "" && item.maxText !== "" && item.tgtText !== "") {
	                                        var min = parseFloat(item.minText); if(isNaN(min)) min = 0.0;
	                                        var max = parseFloat(item.maxText); if(isNaN(max)) max = 0.0;
	                                        var tgt = parseFloat(item.tgtText); if(isNaN(tgt)) tgt = 0.0;
	                                        positions[String(item.rowNum)] = [min, max, tgt]
	                                      }
	                                    }
                                }
                                cartridgeController.saveConfig(cfgZoneCard.configKey, JSON.stringify(positions))
                            }
                        }
	                        CBtn { lbl:"↺ Reset"; padV:10; padH:18; fontSize: 18; bg:root.cBtnBaseStart; bgEnd:root.cBtnBaseEnd; bc:root.cBtnBaseBorder; tc:root.cBtnBaseText
	                            onClicked: cartridgeController.getConfig()
	                        }
                    }
                }
            }

            Rectangle {
                id: cfgZoneScroll
                anchors { right: parent.right; top: parent.top; bottom: parent.bottom; rightMargin: 2; topMargin: 8; bottomMargin: 8 }
                width: 4; radius: 2; color: "#0c1726"
                visible: cfgZoneFlick.height < cfgZoneFlick.contentHeight

                Rectangle {
                    width: parent.width; radius: 2; color: root.cAccent
                    height: Math.max(20, cfgZoneFlick.height * (cfgZoneFlick.height / cfgZoneFlick.contentHeight))
                    y: cfgZoneFlick.visibleArea.yPosition * cfgZoneFlick.height
                }
            }
        }

    Timer {
        id: outTrayTimer
        interval: 200000
        repeat: false
        onTriggered: outTrayPopup.open()
    }

    function checkOutTrayTimer() {
        var robotActive = robotController.systemStatus !== "IDLE" && robotController.systemStatus !== "ERROR" && robotController.systemStatus !== "UNKNOWN" && robotController.systemStatus !== "EMERGENCY_STOP";
        var isAuto = root.currentUiMode === "auto";
        var isManualS3 = root.currentUiMode === "manual" && cartridgeController.stateOut.indexOf("S3") !== -1;
        
        if (!robotController.outReady && robotActive && (isAuto || isManualS3)) {
            if (!outTrayTimer.running) {
                outTrayTimer.restart();
            }
        } else {
            outTrayTimer.stop();
            outTrayPopup.close();
        }
    }

    Connections {
        target: robotController
        function onOutReadyChanged() { checkOutTrayTimer(); }
        function onSystemStatusChanged() { checkOutTrayTimer(); }
    }

    Connections {
        target: cartridgeController
        function onCurrentModeChanged() { checkOutTrayTimer(); }
        function onSystemStateChanged() { checkOutTrayTimer(); }
    }

    Popup {
        id: velPopup
        width: 280; height: 370
        anchors.centerIn: parent
        modal: true; focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle {
            color: "#06101d"; border.color: root.cAccent; border.width: 2; radius: 10
        }

        property string inputStr: ""
        property var targetCard: null

        onOpened: { velPopup.inputStr = "" }

        function openForCard(card) {
            velPopup.targetCard = card
            velPopup.inputStr = ""
            velPopup.open()
        }

        function velPopupApply() {
            var v = parseInt(velPopup.inputStr) || (velPopup.targetCard ? velPopup.targetCard.jogVelMms : velDisplay.jogVelMms)
            v = Math.max(1, Math.min(v, 80))
            if (velPopup.targetCard) velPopup.targetCard.jogVelMms = v
            else velDisplay.jogVelMms = v
            velPopup.close()
        }

        function numpadPress(ch) {
            if (ch === "←") {
                if (velPopup.inputStr.length > 0)
                    velPopup.inputStr = velPopup.inputStr.slice(0, -1)
            } else if (ch === "✓") {
                velPopupApply()
            } else {
                if (velPopup.inputStr.length < 3)
                    velPopup.inputStr += ch
            }
        }

        contentItem: Column {
            anchors.fill: parent; anchors.margins: 14; spacing: 7

            Text {
                text: "Đặt tốc độ JOG"
                color: root.cWhiteText; font.pixelSize: 14; font.bold: true
                width: parent.width; horizontalAlignment: Text.AlignHCenter
            }

            Rectangle {
                width: parent.width; height: 44; radius: 6
                color: "#0c1726"; border.color: root.cAccent; border.width: 1
                Row {
                    anchors.centerIn: parent; spacing: 6
                    Text {
                        text: velPopup.inputStr.length > 0 ? velPopup.inputStr : "–"
                        color: root.cWhiteText; font.pixelSize: 26; font.bold: true
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: "mm/s"; color: root.cWhiteText; font.pixelSize: 13
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }

            Text {
                text: "max 80 mm/s — chỉ áp dụng JOG"
                color: root.cWhiteText; font.pixelSize: 9
                width: parent.width; horizontalAlignment: Text.AlignHCenter
            }

            Repeater {
                model: [["7","8","9"],["4","5","6"],["1","2","3"],["←","0","✓"]]
                delegate: Row {
                    property var keys: modelData
                    spacing: 7
                    Repeater {
                        model: keys
                        delegate: Rectangle {
                            width: 79; height: 44; radius: 6
                            color: modelData === "✓" ? root.cGreen
                                 : modelData === "←" ? "#3a2a2a" : "#0c1726"
                            border.color: root.cBorder; border.width: 1
                            Text {
                                anchors.centerIn: parent
                                text: modelData; color: root.cWhiteText
                                font.pixelSize: 20; font.bold: true
                            }
                            MotionMouseArea {
                                anchors.fill: parent
                                onClicked: velPopup.numpadPress(modelData)
                            }
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width; height: 38; radius: 6
                color: root.cBg; border.color: root.cBorder
                Text { anchors.centerIn: parent; text: "Hủy"; color: root.cWhiteText; font.pixelSize: 13 }
                MotionMouseArea { anchors.fill: parent; onClicked: velPopup.close() }
            }
        }
    }

    Popup {
        id: outTrayPopup
        width: 440; height: 220
        anchors.centerIn: parent
        modal: true; focus: true
        closePolicy: Popup.NoAutoClose
        background: Rectangle { color: "#160a09"; radius: 10; border.color: "#f5a623"; border.width: 3 }
        Column {
            anchors.centerIn: parent; spacing: 30
            Text {
                text: "⚠️ CẢNH BÁO CHƯA CÓ KHAY THÀNH PHẨM"
                color: root.cWhiteText; font.pixelSize: 22; font.bold: true
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text {
                text: "Hệ thống đang chờ khay Output lâu hơn 200s.\nĐã cấp khay chưa?"
                color: root.cWhiteText; font.pixelSize: 18; horizontalAlignment: Text.AlignHCenter
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Row {
                spacing: 40; anchors.horizontalCenter: parent.horizontalCenter
                Rectangle {
                    width: 130; height: 46; radius: 6; color: "#b53527"
                    Text { anchors.centerIn: parent; text: "NO"; color: root.cWhiteText; font.bold: true; font.pixelSize: 16 }
                    MotionMouseArea { anchors.fill: parent; onClicked: { outTrayPopup.close(); outTrayTimer.restart(); } }
                }
                Rectangle {
                    width: 130; height: 46; radius: 6; color: "#1f9e86"
                    Text { anchors.centerIn: parent; text: "YES"; color: root.cWhiteText; font.bold: true; font.pixelSize: 16 }
                    MotionMouseArea { anchors.fill: parent; onClicked: { robotController.simulateOutputTrayReady(); outTrayPopup.close(); } }
                }
            }
        }
    }
    }
