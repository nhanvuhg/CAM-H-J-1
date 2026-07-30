import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Item {
    id: inkTab
    property Item focusHost: null

    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    property bool calActive: scaleController.calStatus === "WAITING_WEIGHT" || scaleController.calStatus.startsWith("CONTINUE_CAL")


    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: currentTime = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    }

    Component.onCompleted: {
        loadProfiles()
    }

    function loadProfiles() {
        inkModel.clear()
        cartModel.clear()
        var inks = scaleController.getInkProfiles()
        var carts = scaleController.getCartProfiles()
        for (var i = 0; i < inks.length; i++) inkModel.append(inks[i])
        for (var j = 0; j < carts.length; j++) cartModel.append(carts[j])
    }

    function parseKvPipe(raw) {
        var out = {};
        if (!raw) return out;
        var parts = String(raw).split("|");
        for (var i = 0; i < parts.length; i++) {
            var part = parts[i];
            var idx = part.indexOf("=");
            var alt = part.indexOf(":");
            var pos = idx >= 0 ? idx : alt;
            if (pos >= 0) out[part.substring(0, pos).trim()] = part.substring(pos + 1).trim();
        }
        return out;
    }

    property var inkStatusMap: parseKvPipe(hpController.inkStatus)

    function inkValue(key, fallback) {
        var value = inkStatusMap[key];
        if (value === undefined || value === null || String(value).trim() === "") {
            return fallback === undefined ? "-" : fallback;
        }
        return String(value);
    }

    function inkNumberText(key, decimals, fallback) {
        var n = Number(inkStatusMap[key]);
        return Number.isFinite(n) ? n.toFixed(decimals) : (fallback === undefined ? "-" : fallback);
    }

    function currentInkNameText() {
        var name = inkValue("INK_NAME", "");
        if (name !== "") return name;
        name = inkValue("NAME", "");
        if (name !== "") return name;
        name = inkValue("MATERIAL", "");
        if (name !== "") return name;
        return inkValue("PRODUCT", inkValue("CODE", inkValue("SCAN", "-")));
    }

    function currentInkCodeText() {
        return inkValue("CODE", inkValue("SCAN", "-"));
    }

    function currentInkLotCIText() {
        return inkValue("LOT_CI", "-");
    }

    function currentInkNeedsLotCI() {
        var code = inkValue("CODE", "");
        var scan = inkValue("SCAN", "");
        return (code !== "" || scan !== "") && currentInkLotCIText() === "-";
    }

    function currentInkStatusText() {
        if (inkValue("CODE_LOCKED", "0") === "1") {
            return "LOCKED " + inkValue("CODE_FAILS", "0") + "/" + inkValue("CODE_MAX_FAILS", "5");
        }
        if (inkValue("DEPLETED", "0") === "1") return "EMPTY";
        if (currentInkNeedsLotCI()) return "NEED LOT CI";
        if (inkValue("CODE", "") !== "" || inkValue("SCAN", "") !== "") return "OK";
        return "NO INK";
    }

    function currentInkStatusColor() {
        if (inkValue("CODE_LOCKED", "0") === "1" || inkValue("DEPLETED", "0") === "1") return cDanger;
        if (currentInkNeedsLotCI()) return cWarning;
        if (inkValue("CODE", "") !== "" || inkValue("SCAN", "") !== "") return cSuccess;
        return cMuted;
    }

    function publishInkBatchCommand(value) {
        hpController.publishString("ink_batch_code", JSON.stringify({
            "value": String(value),
            "operator": "QML"
        }));
    }

    function selectInkByScan(code) {
        var wanted = String(code || "").trim().toUpperCase();
        if (wanted === "") return;
        for (var i = 0; i < inkModel.count; i++) {
            var item = inkModel.get(i);
            var scan = String(item.scan_code || "").trim().toUpperCase();
            var name = String(item.ink_name || item.name || "").trim().toUpperCase();
            if (scan === wanted || name === wanted) {
                inkSelector.currentIndex = i;
                return;
            }
        }
    }

    function submitInkCodeFromPanel() {
        var value = String(inkScanInput.text || "").trim();
        if (value === "") {
            inkScanInput.forceActiveFocus();
            return;
        }
        publishInkBatchCommand(value);
        selectInkByScan(value);
        inkScanInput.text = "";
        inkScanInput.forceActiveFocus();
    }

    function submitLotCIFromPanel() {
        var value = String(inkLotCiInput.text || "").trim();
        if (value === "") {
            inkLotCiInput.forceActiveFocus();
            return;
        }
        publishInkBatchCommand("lot_ci:" + value);
        inkLotCiInput.text = "";
        inkLotCiInput.forceActiveFocus();
    }

    ListModel { id: inkModel }
    ListModel { id: cartModel }

    Connections {
        target: scaleController
        function onProfilesChanged() { loadProfiles() }
    }

    property var numpadTarget: null

    // CartridgePage visual tokens — colors only; behavior and layout stay unchanged.
    readonly property color cBg:               "transparent"
    readonly property color cBorder:           "#1affffff"
    readonly property color cFrameBorder:      "#263548"
    readonly property color cFieldBorder:      "#2b4a5c"
    readonly property color cText:             "#ffffff"
    readonly property color cAccent:           "#67d0ff"
    readonly property color cPanel:            "#990d1e32"
    readonly property color cCard:             "#8806101d"
    readonly property color cField:            Qt.rgba(0.06, 0.19, 0.26, 0.82)
    readonly property color cFieldStrong:      "#081627"
    readonly property color cMuted:            "#74899f"
    readonly property color cSubText:          "#ffffff"
    readonly property color cDisabled:         "#74899f"
    readonly property color cSelectedText:     "#04080f"
    readonly property color cActionStart:      "#1a4a6e"
    readonly property color cActionEnd:        "#0c1726"
    readonly property color cActionHoverStart: "#1a4a6e"
    readonly property color cActionHoverEnd:   "#163a52"
    readonly property color cActionPressStart: "#163a52"
    readonly property color cActionPressEnd:   "#04080f"
    readonly property color cResumeStart:      "#1C4D8D"
    readonly property color cResumeEnd:        "#0c1726"
    readonly property color cResumeHoverStart: "#245fa8"
    readonly property color cResumeHoverEnd:   "#163a52"
    readonly property color cResumePressStart: "#173e72"
    readonly property color cResumePressEnd:   "#081627"
    readonly property color cPauseStart:       "#8a4210"
    readonly property color cPauseEnd:         "#E68457"
    readonly property color cPauseHoverStart:  "#a65315"
    readonly property color cPauseHoverEnd:    "#f09a6d"
    readonly property color cPausePressStart:  "#6f350d"
    readonly property color cPausePressEnd:    "#c76c3f"
    readonly property color cPauseBorder:      "#E68457"
    readonly property color cSuccess:          "#3ed0b4"
    readonly property color cDanger:           "#f0735c"
    readonly property color cWarning:          "#f5a623"
    readonly property int titleFont:           20
    readonly property int sectionFont:         18
    readonly property int labelFont:           14
    readonly property int helperFont:          12
    readonly property int tableFont:           13
    readonly property int buttonFont:          16
    readonly property int inputFont:           16
    readonly property int valueFont:           20
    readonly property int weightFont:          56
    readonly property int resultFont:          32

    component InkMetricBox: Rectangle {
        property string title: ""
        property string value: "-"
        property color valueColor: cAccent

        Layout.fillWidth: true
        Layout.preferredHeight: 52
        color: cFieldStrong
        border.color: cFrameBorder
        border.width: 1
        radius: 6

        Column {
            anchors.centerIn: parent
            width: parent.width - 16
            spacing: 3
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: title
                color: cMuted
                font.pixelSize: helperFont
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                width: parent.width
                text: value
                color: valueColor
                font.pixelSize: inputFont
                font.bold: true
                elide: Text.ElideRight
                horizontalAlignment: Text.AlignHCenter
            }
        }
    }

    Rectangle { anchors.fill: parent; color: cBg }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // ── Main Content ───────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.leftMargin: 15
            Layout.rightMargin: 15
            Layout.topMargin: 15
            Layout.bottomMargin: 24
            spacing: 15

            // ════════ LEFT PANEL: Monitor & Calibration ════════
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredWidth: 600
                Layout.fillHeight: true
                color: cPanel
                border.color: cFrameBorder
                border.width: 1
                radius: 8

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 12

                    Text { text: "LIVE WEIGHT DISPLAY"; color: cAccent; font.pixelSize: titleFont; font.bold: true; Layout.alignment: Qt.AlignHCenter }

                    Rectangle { Layout.fillWidth: true; height: 1; color: cBorder }

                    RowLayout {
                        Layout.alignment: Qt.AlignHCenter
                        spacing: 20
                        Text {
                            text: "● LOADCELL: " + scaleController.loadcellStatus
                            color: (scaleController.loadcellStatus == "OK" || scaleController.loadcellStatus == "SIM") ? cSuccess : cDanger
	                            font.pixelSize: labelFont
                            font.bold: true
                        }
                        Text {
                            text: "Scale node: ● " + (scaleController.scaleNodeConnected ? "CONNECTED" : "DISCONNECTED")
                            color: scaleController.scaleNodeConnected ? cSuccess : cDanger
	                            font.pixelSize: labelFont
                            font.bold: true
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 110
                        color: cFieldStrong
                        border.color: cFieldBorder
                        border.width: 1
                        radius: 8

                        Text {
                            anchors.centerIn: parent
                            text: scaleController.currentWeight.toFixed(1) + " g"
                            color: "#ffffff"
	                            font.pixelSize: weightFont
                            font.bold: true
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

	                        Text { text: "Status:"; color: cSubText; font.pixelSize: labelFont }
                        Rectangle {
                            width: 140; height: 35; radius: 6
                            color: {
                                var s = scaleController.monitorStatus;
                                if(s === "NO_SIGNAL") return cDisabled;
                                if(s === "MEASURING") return cWarning;
                                if(s === "PASS") return cSuccess;
                                if(s === "FAIL") return cDanger;
                                return cDisabled;
                            }
	                            Text { anchors.centerIn: parent; text: scaleController.monitorStatus; color: "#fff"; font.pixelSize: labelFont; font.bold: true }
                        }

                        Item { Layout.fillWidth: true }

	                        Text { text: "Profile:"; color: cSubText; font.pixelSize: labelFont }
	                        Text { text: scaleController.activeInkName === "NONE" ? "NOT SELECTED" : scaleController.activeInkName; color: cAccent; font.pixelSize: valueFont; font.bold: true }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 50
                        Layout.maximumHeight: 50
                        spacing: 15

                        MotionButton {
                            id: tareBtn
                            opacity: down ? 0.8 : 1.0
                            text: "TARE"
                            Layout.fillWidth: true; Layout.preferredHeight: 50; Layout.maximumHeight: 50
	                            font.pixelSize: buttonFont; font.bold: true
                            onClicked: scaleController.tare()
                            background: Rectangle {
                                radius: 6
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: tareBtn.down ? cActionPressStart : (tareBtn.hovered ? cActionHoverStart : cActionStart) }
                                    GradientStop { position: 1.0; color: tareBtn.down ? cActionPressEnd : (tareBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                }
                            }
                            contentItem: Text {
                                text: tareBtn.text
                                color: "#ffffff"
                                font: tareBtn.font
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }
                        }
                        MotionButton {
                            id: resetTareBtn
                            opacity: down ? 0.8 : 1.0
                            text: "RESET TARE"
                            Layout.fillWidth: true; Layout.preferredHeight: 50; Layout.maximumHeight: 50
	                            font.pixelSize: buttonFont; font.bold: true
                            onClicked: scaleController.resetTare()
                            background: Rectangle {
                                radius: 6
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: resetTareBtn.down ? cPausePressStart : (resetTareBtn.hovered ? cPauseHoverStart : cPauseStart) }
                                    GradientStop { position: 1.0; color: resetTareBtn.down ? cPausePressEnd : (resetTareBtn.hovered ? cPauseHoverEnd : cPauseEnd) }
                                }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: cBorder }

                    // ── MOVED: CALIBRATE SCALE ──
                    RowLayout {
                        Layout.fillWidth: true
	                        Text { text: "CALIBRATE SCALE"; color: cAccent; font.pixelSize: sectionFont; font.bold: true }
                        Item { Layout.fillWidth: true }
	                        Text { text: "Status: " + scaleController.calStatus; color: cWarning; font.pixelSize: labelFont; font.bold: true }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 70
                        color: cCard
                        border.color: cFrameBorder
                        radius: 6
                        RowLayout {
                            anchors.fill: parent; anchors.margins: 10
                            ColumnLayout {
	                                Text { text: "STEP 1 — Empty Scale"; color: "#ecc45a"; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: "Ensure NOTHING is on the scale."; color: "#c7dcef"; font.pixelSize: helperFont }
                            }
                            Item { Layout.fillWidth: true }
                            MotionButton {
                                id: setZeroBtn
                                opacity: down ? 0.8 : 1.0
                                text: "SET ZERO"
                                Layout.preferredWidth: 120; Layout.preferredHeight: 35
	                                font.pixelSize: labelFont; font.bold: true
                                onClicked: scaleController.startCalibration()
                                background: Rectangle {
                                    radius: 4
                                    gradient: Gradient {
                                        orientation: Gradient.Horizontal
                                        GradientStop { position: 0.0; color: setZeroBtn.down ? cActionPressStart : (setZeroBtn.hovered ? cActionHoverStart : cActionStart) }
                                        GradientStop { position: 1.0; color: setZeroBtn.down ? cActionPressEnd : (setZeroBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                    }
                                }
                                contentItem: Text {
                                    text: setZeroBtn.text; font: setZeroBtn.font
                                    color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                }
                                enabled: scaleController.calStatus === "IDLE" || scaleController.calStatus === "ERROR"
                            }
                        }
                    }



                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 130
                        color: cCard
                        border.color: calActive ? cSuccess : cBorder
                        radius: 6
                        ColumnLayout {
                            anchors.fill: parent; anchors.margins: 10; spacing: 4
                            RowLayout {
                                Layout.fillWidth: true
                                ColumnLayout {
                                    spacing: 2
                                    Text {
                                        text: {
                                            if (scaleController.calStatus === "WAITING_WEIGHT") return "STEP 2 — Place Standard Weight (Point 1/4)";
                                            if (scaleController.calStatus === "CONTINUE_CAL_2/5") return "STEP 3 — Place Next Weight (Point 2/4)";
                                            if (scaleController.calStatus === "CONTINUE_CAL_3/5") return "STEP 4 — Place Next Weight (Point 3/4)";
                                            if (scaleController.calStatus === "CONTINUE_CAL_4/5") return "STEP 5 — Place Final Weight (Point 4/4)";
                                            return "STEP 2 — Place Standard Weight";
                                        }
                                        color: "#ecc45a"
	                                        font.pixelSize: tableFont; font.bold: true
                                    }
                                    Text {
                                        text: {
                                            if (scaleController.calStatus === "WAITING_WEIGHT") return "Suggested: 100g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_2/5") return "Suggested: 250g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_3/5") return "Suggested: 500g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_4/5") return "Suggested: 1000g";
                                            return "Enter known weight below";
                                        }
	                                        color: cSubText; font.pixelSize: helperFont
                                    }
                                }
                                Item { Layout.fillWidth: true }
                                // Progress dots
                                Row {
                                    spacing: 5
                                    Repeater {
                                        model: 4
                                        Rectangle {
                                            width: 10; height: 10; radius: 5
                                            color: {
                                                var n = index + 1;
                                                if (scaleController.calStatus === "DONE") return cSuccess;
                                                if (scaleController.calStatus === "CONTINUE_CAL_2/5" && n <= 1) return cSuccess;
                                                if (scaleController.calStatus === "CONTINUE_CAL_3/5" && n <= 2) return cSuccess;
                                                if (scaleController.calStatus === "CONTINUE_CAL_4/5" && n <= 3) return cSuccess;
                                                var activeIndex = 0;
                                                var s = scaleController.calStatus;
                                                if (s === "WAITING_WEIGHT") activeIndex = 1;
                                                else if (s === "CONTINUE_CAL_2/5") activeIndex = 2;
                                                else if (s === "CONTINUE_CAL_3/5") activeIndex = 3;
                                                else if (s === "CONTINUE_CAL_4/5") activeIndex = 4;

                                                if ((s === "WAITING_WEIGHT" || s.startsWith("CONTINUE_CAL")) && n === activeIndex) return cWarning;
                                                return "#14263c";
                                            }
                                        }
                                    }
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                TextField {
                                    id: tfCalW
                                    placeholderText: "Enter value (g)"; placeholderTextColor: cSubText
                                    Layout.fillWidth: true; Layout.preferredHeight: 45
	                                    font.pixelSize: valueFont; font.bold: true; color: cWarning
                                    horizontalAlignment: TextInput.AlignHCenter
                                    verticalAlignment: TextInput.AlignVCenter
                                    validator: DoubleValidator{}
                                    text: scaleController.lastKnownCalWeight.toString()
                                    enabled: calActive
                                    readOnly: true
                                    background: Rectangle { color: cField; radius: 6; border.color: cFieldBorder; border.width: 1 }
                                    MotionMouseArea {
                                        anchors.fill: parent
                                        onClicked: {
                                            inkTab.numpadTarget = tfCalW;
                                            numpadPopup.currentValue = tfCalW.text;
                                            numpadPopup.open();
                                        }
                                    }
                                }
                                MotionButton {
                                    id: applyStep2Btn
                                    opacity: down ? 0.8 : 1.0
                                    text: scaleController.calStatus === "CONTINUE_CAL_4/5" ? "FINISH" : "APPLY"
                                    Layout.preferredWidth: 120; Layout.preferredHeight: 45
	                                    font.pixelSize: buttonFont; font.bold: true
                                    onClicked: {
                                        scaleController.setLastKnownCalWeight(parseFloat(tfCalW.text))
                                        scaleController.setKnownCalibration(scaleController.lastKnownCalWeight)
                                    }
                                    background: Rectangle {
                                        radius: 6
                                        gradient: Gradient {
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: calActive ? (applyStep2Btn.down ? cActionPressStart : (applyStep2Btn.hovered ? cActionHoverStart : cActionStart)) : cDisabled }
                                            GradientStop { position: 1.0; color: calActive ? (applyStep2Btn.down ? cActionPressEnd : (applyStep2Btn.hovered ? cActionHoverEnd : cActionEnd)) : cDisabled }
                                        }
                                    }
                                    contentItem: Text {
                                        text: applyStep2Btn.text; font: applyStep2Btn.font
                                        color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                    }
                                    enabled: calActive
                                }
                            }
                        }
                    }
                    Item { Layout.fillHeight: true } // SPACER TO PUSH BATCH DOWN

                    Rectangle { Layout.fillWidth: true; height: 2; color: cBorder }

	                    Text { text: "PRODUCTION RESULTS"; color: cAccent; font.pixelSize: sectionFont; font.bold: true; font.letterSpacing: 2; Layout.alignment: Qt.AlignHCenter }

                    // ── STAT CARDS ROW ──
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        // TOTAL card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
	                            color: cCard; border.color: cFrameBorder; border.width: 1
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
	                                Text { text: "TOTAL"; color: cSubText; font.pixelSize: labelFont; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
	                                Text { text: scaleController.totalBatch.toString(); color: "#ffffff"; font.pixelSize: resultFont; font.bold: true; Layout.alignment: Qt.AlignHCenter }
	                                Text { text: "batch"; color: "#bfe0f5"; font.pixelSize: helperFont; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                        // PASS card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
	                            color: Qt.rgba(0.13, 0.77, 0.37, 0.15); border.color: cFrameBorder; border.width: 1
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
	                                Text { text: "✓ PASS"; color: "#3ed0b4"; font.pixelSize: labelFont; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
	                                Text { text: scaleController.passBatch.toString(); color: cSuccess; font.pixelSize: resultFont; font.bold: true; Layout.alignment: Qt.AlignHCenter }
	                                Text { text: "batch"; color: "#1f9e86"; font.pixelSize: helperFont; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                        // FAIL card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
	                            color: Qt.rgba(0.94, 0.27, 0.27, 0.15); border.color: cFrameBorder; border.width: 1
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
	                                Text { text: "✗ FAIL"; color: "#f5a394"; font.pixelSize: labelFont; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
	                                Text { text: scaleController.failBatch.toString(); color: cDanger; font.pixelSize: resultFont; font.bold: true; Layout.alignment: Qt.AlignHCenter }
	                                Text { text: "batch"; color: "#b53527"; font.pixelSize: helperFont; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                    }

                    // ── FAIL STREAK + RESET ──
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 48; radius: 6
                            color: scaleController.consecFails >= 3 ? Qt.rgba(0.94, 0.27, 0.27, 0.15) : cCard
                            border.color: scaleController.consecFails >= 3 ? cDanger : cBorder; border.width: 1
                            RowLayout {
                                anchors.centerIn: parent; spacing: 8
	                                Text { text: "⚠ CONSECUTIVE FAILS:"; color: cSubText; font.pixelSize: labelFont; font.bold: true; font.letterSpacing: 1 }
	                                Text { text: scaleController.consecFails.toString(); color: scaleController.consecFails >= 3 ? cDanger : cWarning; font.pixelSize: valueFont; font.bold: true }
                            }
                        }
                        MotionButton {
                            id: resetBatchBtn
                            opacity: down ? 0.8 : 1.0
                            text: "RESET BATCH"
                            Layout.preferredWidth: 130; Layout.preferredHeight: 48
	                            font.pixelSize: buttonFont; font.bold: true
                            onClicked: scaleController.resetBatch()
                            background: Rectangle {
                                radius: 6
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: resetBatchBtn.down ? cActionPressStart : (resetBatchBtn.hovered ? cActionHoverStart : cActionStart) }
                                    GradientStop { position: 1.0; color: resetBatchBtn.down ? cActionPressEnd : (resetBatchBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                }
                            }
                            contentItem: Text {
                                text: parent.text; font: parent.font
                                color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                            }
                        }
                    }
                }
            }

            // ════════ RIGHT PANEL: Select Profile & Create New ════════
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredWidth: 600
                Layout.fillHeight: true
                color: cPanel
                border.color: cFrameBorder
                border.width: 1
                radius: 8

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 15

                    // Title
	                    Text { text: "INK PROFILES"; color: "#67d0ff"; font.pixelSize: titleFont; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }

	                    // ── MỤC CHỌN MỰC TƯƠNG TỰ 'SELECT MODE' ──
	                    Rectangle { Layout.fillWidth: true; height: 1; color: cBorder; opacity: 0.5 }

	                    Rectangle {
	                        Layout.fillWidth: true
	                        Layout.preferredHeight: 268
	                        color: cCard
	                        border.color: cFrameBorder
	                        border.width: 1
	                        radius: 8

	                        ColumnLayout {
	                            anchors.fill: parent
	                            anchors.margins: 12
	                            spacing: 8

	                            RowLayout {
	                                Layout.fillWidth: true
	                                spacing: 10

	                                Text {
	                                    text: "INK BATCH STATUS"
	                                    color: cAccent
	                                    font.pixelSize: sectionFont
	                                    font.bold: true
	                                    font.letterSpacing: 1
	                                }
	                                Item { Layout.fillWidth: true }
	                                Text {
	                                    text: currentInkNameText()
	                                    color: "#ffffff"
	                                    font.pixelSize: valueFont
	                                    font.bold: true
	                                    elide: Text.ElideRight
	                                    Layout.maximumWidth: 260
	                                }
	                            }

	                            Rectangle {
	                                Layout.fillWidth: true
	                                Layout.preferredHeight: 44
	                                color: cField
	                                border.color: cFieldBorder
	                                border.width: 1
	                                radius: 7

	                                RowLayout {
	                                    anchors.fill: parent
	                                    anchors.leftMargin: 16
	                                    anchors.rightMargin: 16
	                                    spacing: 12
	                                    Text {
	                                        text: "CODE"
	                                        color: cMuted
	                                        font.pixelSize: tableFont
	                                        font.bold: true
	                                    }
	                                    Text {
	                                        text: currentInkCodeText()
	                                        color: cAccent
	                                        font.pixelSize: valueFont
	                                        font.bold: true
	                                        Layout.preferredWidth: 130
	                                        elide: Text.ElideRight
	                                    }
	                                    Item { Layout.fillWidth: true }
	                                    Text {
	                                        text: inkNumberText("REMAIN", 2)
	                                        color: "#ffffff"
	                                        font.pixelSize: valueFont
	                                        font.bold: true
	                                    }
	                                    Text {
	                                        text: "/ " + inkNumberText("TOTAL", 2) + " kg remaining"
	                                        color: cSubText
	                                        font.pixelSize: labelFont
	                                        font.bold: true
	                                    }
	                                }
	                            }

	                            GridLayout {
	                                Layout.fillWidth: true
	                                columns: 5
	                                columnSpacing: 8
	                                rowSpacing: 8

	                                InkMetricBox { title: "LEFT (BATCH)"; value: inkNumberText("REMAIN_BATCHES", 2) }
	                                InkMetricBox { title: "NEED/BATCH (g)"; value: inkNumberText("BATCH_NEED_G", 1) }
	                                InkMetricBox { title: "STATUS"; value: currentInkStatusText(); valueColor: currentInkStatusColor() }
	                                InkMetricBox { title: "LOT PI"; value: inkValue("LOT_PI", "-") }
	                                InkMetricBox { title: "LOT CI"; value: currentInkLotCIText(); valueColor: currentInkNeedsLotCI() ? cWarning : cAccent }
	                            }

	                            RowLayout {
	                                Layout.fillWidth: true
	                                spacing: 8

	                                Rectangle {
	                                    Layout.fillWidth: true
	                                    Layout.preferredHeight: 40
	                                    color: cField
	                                    border.color: cFieldBorder
	                                    border.width: 1
	                                    radius: 6
	                                    SmartTextField {
	                                        id: inkScanInput
	                                        focusHost: inkTab.focusHost
	                                        anchors.fill: parent
	                                        anchors.margins: 1
	                                        leftPadding: 11
	                                        rightPadding: 11
	                                        color: "#ffffff"
	                                        placeholderTextColor: cMuted
	                                        selectionColor: cAccent
	                                        selectedTextColor: cSelectedText
	                                        font.pixelSize: inputFont
	                                        font.bold: true
	                                        verticalAlignment: TextInput.AlignVCenter
	                                        clip: true
	                                        selectByMouse: true
	                                        placeholderText: "Enter / scan ink code"
	                                        background: Rectangle { color: "transparent" }
	                                        Keys.onReturnPressed: submitInkCodeFromPanel()
	                                        Keys.onEnterPressed: submitInkCodeFromPanel()
	                                    }
	                                }

	                                MotionButton {
	                                    id: submitInkBtn
	                                    text: "APPLY"
	                                    Layout.preferredWidth: 86
	                                    Layout.preferredHeight: 40
	                                    font.pixelSize: labelFont
	                                    font.bold: true
	                                    onClicked: submitInkCodeFromPanel()
	                                    background: Rectangle {
	                                        radius: 6
	                                        gradient: Gradient {
	                                            orientation: Gradient.Horizontal
	                                            GradientStop { position: 0.0; color: submitInkBtn.down ? cActionPressStart : (submitInkBtn.hovered ? cActionHoverStart : cActionStart) }
	                                            GradientStop { position: 1.0; color: submitInkBtn.down ? cActionPressEnd : (submitInkBtn.hovered ? cActionHoverEnd : cActionEnd) }
	                                        }
	                                    }
	                                    contentItem: Text { text: parent.text; font: parent.font; color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
	                                }

	                                MotionButton {
	                                    id: outInkBtn
	                                    text: "CLEAR"
	                                    Layout.preferredWidth: 70
	                                    Layout.preferredHeight: 40
	                                    font.pixelSize: labelFont
	                                    font.bold: true
	                                    onClicked: { publishInkBatchCommand("out"); inkScanInput.forceActiveFocus(); }
	                                    background: Rectangle {
	                                        radius: 6
	                                        color: cFieldStrong
	                                        border.color: cFieldBorder
	                                        border.width: 1
	                                    }
	                                    contentItem: Text { text: parent.text; font: parent.font; color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
	                                }
	                            }

	                            Rectangle {
	                                Layout.fillWidth: true
	                                Layout.preferredHeight: 48
	                                color: cFieldStrong
	                                border.color: cFrameBorder
	                                border.width: 1
	                                radius: 7

	                                RowLayout {
	                                    anchors.fill: parent
	                                    anchors.margins: 8
	                                    spacing: 8
	                                    Text {
	                                        text: "LOT CI"
	                                        color: currentInkNeedsLotCI() ? cWarning : cAccent
	                                        font.pixelSize: tableFont
	                                        font.bold: true
	                                        Layout.preferredWidth: 62
	                                    }
		                                    Rectangle {
		                                        Layout.fillWidth: true
		                                        Layout.preferredHeight: 34
	                                        color: cField
	                                        border.color: cFieldBorder
	                                        border.width: 1
	                                        radius: 5
		                                        SmartTextField {
		                                            id: inkLotCiInput
		                                            focusHost: inkTab.focusHost
		                                            anchors.fill: parent
		                                            anchors.margins: 1
		                                            leftPadding: 11
		                                            rightPadding: 11
		                                            color: "#ffffff"
		                                            placeholderTextColor: cMuted
		                                            selectionColor: cAccent
		                                            selectedTextColor: cSelectedText
		                                            font.pixelSize: inputFont
		                                            font.bold: true
		                                            verticalAlignment: TextInput.AlignVCenter
		                                            clip: true
		                                            selectByMouse: true
		                                            placeholderText: "Enter Lot CI"
		                                            background: Rectangle { color: "transparent" }
		                                            Keys.onReturnPressed: submitLotCIFromPanel()
		                                            Keys.onEnterPressed: submitLotCIFromPanel()
		                                        }
		                                    }
	                                    MotionButton {
	                                        id: confirmLotCiBtn
		                                        text: "ACCEPT LOT CI"
		                                        Layout.preferredWidth: 150
		                                        Layout.preferredHeight: 34
	                                        font.pixelSize: tableFont
	                                        font.bold: true
	                                        onClicked: submitLotCIFromPanel()
	                                        background: Rectangle {
	                                            radius: 6
	                                            gradient: Gradient {
	                                                orientation: Gradient.Horizontal
	                                                GradientStop { position: 0.0; color: confirmLotCiBtn.down ? cActionPressStart : (confirmLotCiBtn.hovered ? cActionHoverStart : cActionStart) }
	                                                GradientStop { position: 1.0; color: confirmLotCiBtn.down ? cActionPressEnd : (confirmLotCiBtn.hovered ? cActionHoverEnd : cActionEnd) }
	                                            }
	                                        }
	                                        contentItem: Text { text: parent.text; font: parent.font; color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
	                                    }
	                                }
	                            }
	                        }
	                    }

	                    RowLayout {
	                        Layout.fillWidth: true
                        spacing: 20

	                        // Cột 1: Chọn Mực
	                        ColumnLayout {
	                            Layout.fillWidth: true
	                            Layout.preferredWidth: 560
	                            Text { text: "SELECT INK PROFILE"; color: "#67d0ff"; font.pixelSize: tableFont; font.bold: true; font.letterSpacing: 1 }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 4
                                color: cField; border.color: cFieldBorder; border.width: 1
	                                ComboBox {
	                                    id: inkSelector
	                                    anchors.fill: parent; anchors.margins: 1
	                                    model: inkModel
	                                    textRole: "display"
	                                    font.pixelSize: labelFont; font.bold: true
                                    background: Rectangle { color: "transparent" }
                                    indicator: Image {
                                        x: inkSelector.width - width - 10
                                        y: (inkSelector.height - height) / 2
                                        width: 24; height: 24
                                        source: "icons/folder_search.svg"
                                        sourceSize.width: 96
                                        sourceSize.height: 96
                                        fillMode: Image.PreserveAspectFit
                                        smooth: true
                                        mipmap: true
                                        antialiasing: true
                                    }
                                    contentItem: Text { text: inkSelector.currentIndex >= 0 ? inkSelector.displayText : "-- Select Ink --"; font: inkSelector.font; color: cAccent; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter }
                                    popup: Popup {
                                        y: inkSelector.height; width: inkSelector.width; implicitHeight: contentItem.implicitHeight + 36; padding: 0
                                        background: Rectangle { color: cField; border.color: cFieldBorder; border.width: 1; radius: 4 }
                                        contentItem: Column {
                                            width: parent.width
                                            // Table Header
                                            Rectangle {
	                                                width: parent.width; height: 32; color: cCard
                                                Row {
	                                                    anchors.fill: parent; spacing: 0
	                                                    Item { width: parent.width * 0.07; height: parent.height; Text { anchors.centerIn: parent; text: "No"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.16 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: "SCAN CODE"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: "INK NAME"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.13 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "TOTAL KG"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.12 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DENSITY"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.34 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: "LOT PI"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                }
	                                            }
                                            Rectangle { width: parent.width; height: 2; color: cAccent }
                                            // Table Rows
                                            ListView {
                                                width: parent.width; implicitHeight: contentHeight; clip: true
                                                model: inkSelector.delegateModel
                                            }
                                        }
                                    }
                                    delegate: ItemDelegate {
	                                        width: inkSelector.width; height: 38
                                        contentItem: Item {
                                            anchors.fill: parent
	                                            Row {
	                                                anchors.fill: parent; spacing: 0
	                                                Item { width: parent.width * 0.07; height: parent.height; Text { anchors.centerIn: parent; text: (index+1).toString(); color: inkSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item { width: parent.width * 0.16 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: model.scan_code || "--"; color: inkSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true; elide: Text.ElideRight; width: parent.width - 12 } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: model.ink_name || model.name || "--"; color: inkSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true; elide: Text.ElideRight; width: parent.width - 12 } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item { width: parent.width * 0.13 - 1; height: parent.height; Text { anchors.centerIn: parent; text: Number(model.total_kg || 0).toFixed(2); color: inkSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item { width: parent.width * 0.12 - 1; height: parent.height; Text { anchors.centerIn: parent; text: Number(model.density || 0).toFixed(2); color: inkSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item { width: parent.width * 0.34 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: model.lot_pi || "--"; color: inkSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true; elide: Text.ElideRight; width: parent.width - 12 } }
	                                            }
                                            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: cBorder }
                                        }
                                        background: Rectangle { color: inkSelector.highlightedIndex === index ? cAccent : cField }
                                    }
                                }
                            }
                        }

	                        // Cột 2: Chọn Vỏ
	                        ColumnLayout {
	                            Layout.fillWidth: false
	                            Layout.preferredWidth: 340
	                            Layout.maximumWidth: 360
	                            Text { text: "SELECT CARTRIDGE TYPE"; color: cAccent; font.pixelSize: tableFont; font.bold: true; font.letterSpacing: 1 }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 4
                                color: cField; border.color: cFieldBorder; border.width: 1
                                ComboBox {
                                    id: cartSelector
                                    anchors.fill: parent; anchors.margins: 1
                                    model: cartModel
                                    textRole: "name"
	                                    font.pixelSize: labelFont; font.bold: true
                                    background: Rectangle { color: "transparent" }
                                    indicator: Image {
                                        x: cartSelector.width - width - 10
                                        y: (cartSelector.height - height) / 2
                                        width: 24; height: 24
                                        source: "icons/folder_search.svg"
                                        sourceSize.width: 96
                                        sourceSize.height: 96
                                        fillMode: Image.PreserveAspectFit
                                        smooth: true
                                        mipmap: true
                                        antialiasing: true
                                    }
                                    contentItem: Text { text: cartSelector.currentIndex >= 0 ? cartSelector.displayText : "-- Select Cartridge --"; font: cartSelector.font; color: cAccent; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter }
                                    popup: Popup {
                                        y: cartSelector.height; width: cartSelector.width; implicitHeight: contentItem.implicitHeight + 36; padding: 0
                                        background: Rectangle { color: cField; border.color: cFieldBorder; border.width: 1; radius: 4 }
                                        contentItem: Column {
                                            width: parent.width
                                            Rectangle {
	                                                width: parent.width; height: 32; color: cCard
                                                Row {
                                                    anchors.fill: parent; spacing: 0
	                                                    Item { width: parent.width * 0.14; height: parent.height; Text { anchors.centerIn: parent; text: "No"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.46 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: "CART NAME"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.30 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "CART WEIGHT"; color: cAccent; font.pixelSize: tableFont; font.bold: true } }
	                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                    Item { width: parent.width * 0.10 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DEL"; color: cDanger; font.pixelSize: tableFont; font.bold: true } }
                                                }
                                            }
                                            Rectangle { width: parent.width; height: 2; color: cAccent }
                                            ListView {
                                                width: parent.width; implicitHeight: contentHeight; clip: true
                                                model: cartSelector.delegateModel
                                            }
                                        }
                                    }
                                    delegate: ItemDelegate {
	                                        width: cartSelector.width; height: 38
                                        contentItem: Item {
                                            anchors.fill: parent
                                            Row {
                                                anchors.fill: parent; spacing: 0
	                                                Item { width: parent.width * 0.14; height: parent.height; Text { anchors.centerIn: parent; text: (index+1).toString(); color: cartSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item { width: parent.width * 0.46 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: model.name; color: cartSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true; elide: Text.ElideRight; width: parent.width - 12 } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item { width: parent.width * 0.30 - 1; height: parent.height; Text { anchors.centerIn: parent; text: model.density.toFixed(2) + " g"; color: cartSelector.highlightedIndex === index ? cSelectedText : cText; font.pixelSize: labelFont; font.bold: true } }
	                                                Rectangle { width: 1; height: parent.height; color: cBorder }
	                                                Item {
	                                                    width: parent.width * 0.10 - 1; height: parent.height
                                                    Rectangle {
                                                        anchors.centerIn: parent; width: 22; height: 22; radius: 4; color: delCartMA.pressed ? "#b53527" : cDanger
	                                                        Text { anchors.centerIn: parent; text: "✕"; font.bold: true; font.pixelSize: helperFont; color: "#fff" }
                                                        MotionMouseArea { id: delCartMA; anchors.fill: parent; onClicked: { scaleController.deleteCartProfile(model.name); } }
                                                    }
                                                }
                                            }
                                            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: cBorder }
                                        }
                                        background: Rectangle { color: cartSelector.highlightedIndex === index ? cAccent : cField }
                                    }
                                }
                            }
                        }
                    }

                    // --- DETAILS AREA ---
	                    Rectangle {
	                        Layout.fillWidth: true
	                        Layout.preferredHeight: 300
	                        color: cCard; border.color: cFrameBorder; border.width: 1; radius: 6

                        GridLayout {
                            anchors.fill: parent; anchors.margins: 15
                            columns: 3; rowSpacing: 15; columnSpacing: 30

	                            // Row 1
	                            RowLayout {
	                                Text { text: "SCAN CODE:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: inkSelector.currentIndex >= 0 ? inkModel.get(inkSelector.currentIndex).scan_code : "--"; color: cAccent; font.pixelSize: valueFont; font.bold: true }
	                            }
	                            RowLayout {
	                                Text { text: "INK NAME:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: inkSelector.currentIndex >= 0 ? (inkModel.get(inkSelector.currentIndex).ink_name || inkModel.get(inkSelector.currentIndex).name) : "--"; color: cAccent; font.pixelSize: valueFont; font.bold: true }
	                            }
	                            RowLayout {
	                                Text { text: "TOTAL KG:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: inkSelector.currentIndex >= 0 ? Number(inkModel.get(inkSelector.currentIndex).total_kg || 0).toFixed(2) : "--"; color: cAccent; font.pixelSize: valueFont; font.bold: true }
	                            }

	                            // Row 2
	                            RowLayout {
	                                Text { text: "DENSITY:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: inkSelector.currentIndex >= 0 ? Number(inkModel.get(inkSelector.currentIndex).density || 0).toFixed(2) + " g/ml" : "--"; color: cAccent; font.pixelSize: valueFont; font.bold: true }
	                            }
	                            RowLayout {
	                                Text { text: "LOT PI:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: inkSelector.currentIndex >= 0 ? (inkModel.get(inkSelector.currentIndex).lot_pi || "--") : "--"; color: cAccent; font.pixelSize: valueFont; font.bold: true }
	                            }
	                            RowLayout {
			                                Text { text: "RELATIVE ERROR (g):"; color: cWarning; font.pixelSize: labelFont; font.bold: true }
		                                Rectangle {
		                                    width: 70; height: 35; color: cField; border.color: cWarning; border.width: 0.5; radius: 4
		                                    TextInput {
		                                        id: relativeErrorInput
			                                        anchors.fill: parent; anchors.margins: 2; color: cWarning; font.pixelSize: inputFont; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
	                                        text: "1.0"
	                                        readOnly: true
	                                        MotionMouseArea { anchors.fill: parent; onClicked: { inkTab.numpadTarget = relativeErrorInput; numpadPopup.currentValue = relativeErrorInput.text; numpadPopup.open() } }
	                                    }
	                                }
	                            }

	                            // Row 3
	                            RowLayout {
	                                Text { text: "CARTRIDGE TYPE:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: cartSelector.currentIndex >= 0 ? cartModel.get(cartSelector.currentIndex).name : "--"; color: cAccent; font.pixelSize: valueFont; font.bold: true }
	                            }
	                            RowLayout {
	                                Text { text: "CART WEIGHT:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
	                                Text { text: cartSelector.currentIndex >= 0 ? Number(cartModel.get(cartSelector.currentIndex).density || 0).toFixed(2) + " g" : "--"; color: cAccent; font.pixelSize: valueFont; font.bold: true }
	                            }
                            RowLayout {
                                Text { text: "CART WEIGHT ERROR (g):"; color: cWarning; font.pixelSize: labelFont; font.bold: true }
                                Rectangle {
                                    width: 70; height: 35; color: cField; border.color: cWarning; border.width: 0.5; radius: 4
                                    TextInput {
                                        id: cartWeightErrorInput
                                        anchors.fill: parent; anchors.margins: 2
                                        color: cWarning
                                        font.pixelSize: inputFont
                                        font.bold: true
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                        text: "0.0"
                                        readOnly: true
                                        MotionMouseArea {
                                            anchors.fill: parent
                                            onClicked: {
                                                inkTab.numpadTarget = cartWeightErrorInput;
                                                numpadPopup.currentValue = cartWeightErrorInput.text;
                                                numpadPopup.open();
                                            }
                                        }
                                    }
                                }
                            }

	                            // Row 4
                            RowLayout {
                                Text { text: "CURRENT ML FILL:"; color: cSubText; font.pixelSize: labelFont; font.bold: true }
                                Rectangle {
                                    width: 80; height: 35; color: cField; border.color: cFieldBorder; border.width: 1; radius: 4
                                    Text {
                                        anchors.centerIn: parent; text: scaleController.currentMlFill.toFixed(1) + " ml"; color: cSuccess; font.pixelSize: inputFont; font.bold: true
                                    }
                                }
                            }
                            RowLayout {
                                Text { text: "TYPE WEIGHT FILL:"; color: cAccent; font.pixelSize: labelFont; font.bold: true }
                                Rectangle {
                                    width: 70; height: 35; color: cField; border.color: cFieldBorder; border.width: 1; radius: 4
                                    TextInput {
                                        id: inkCapacityInput
                                        anchors.fill: parent; anchors.margins: 2; color: cAccent; font.pixelSize: inputFont; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                        text: scaleController.currentMlFill.toFixed(1)
                                        readOnly: true
                                        MotionMouseArea { anchors.fill: parent; onClicked: { inkTab.numpadTarget = inkCapacityInput; numpadPopup.currentValue = inkCapacityInput.text; numpadPopup.open() } }
                                        Connections {
                                            target: scaleController
                                            function onCurrentMlFillChanged() {
                                                if (inkTab.numpadTarget !== inkCapacityInput) {
                                                    inkCapacityInput.text = scaleController.currentMlFill.toFixed(1);
                                                }
                                            }
                                        }
                                        onTextChanged: {
                                            var testVal = parseFloat(text.replace(",", ".")) || 0.0;
                                            if (testVal > 70.0) {
                                                text = "70.0";
                                            }
                                        }
                                    }
                                }
                            }
                            Item { Layout.fillWidth: true }

                            // Row 4: CONFIRM BTN & CLEAR SELECTION
                            RowLayout {
                                Layout.columnSpan: 3; Layout.fillWidth: true; spacing: 15
                                Item { Layout.fillWidth: true }
                                MotionButton {
                                    id: applyTargetBtn
                                    opacity: down ? 0.8 : 1.0
                                    text: "APPLY TARGET (RUN)"
                                    Layout.preferredHeight: 50; Layout.preferredWidth: 240
                                    background: Rectangle {
                                        radius: 5
                                        gradient: Gradient {
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: applyTargetBtn.down ? cActionPressStart : (applyTargetBtn.hovered ? cActionHoverStart : cActionStart) }
                                            GradientStop { position: 1.0; color: applyTargetBtn.down ? cActionPressEnd : (applyTargetBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                        }
                                    }
	                                    contentItem: Text { text: parent.text; font.pixelSize: buttonFont; font.bold: true; color: "#ffffff"; horizontalAlignment: Qt.AlignHCenter; verticalAlignment: Qt.AlignVCenter }
	                                    onClicked: {
	                                        if (inkSelector.currentIndex >= 0 && cartSelector.currentIndex >= 0) {
	                                            var selectedInk = inkModel.get(inkSelector.currentIndex);
	                                            var scanCode = String(selectedInk.scan_code || "").trim();
	                                            var inkN = selectedInk.ink_name || selectedInk.name;
	                                            var inkD = Number(selectedInk.density || selectedInk.density_g_ml || 0.89);
		                                            var cartN = cartModel.get(cartSelector.currentIndex).name;
		                                            var cartD = Number(cartModel.get(cartSelector.currentIndex).density || 0.0);
		                                            var relE = parseFloat(relativeErrorInput.text.replace(",", ".")) || 0.0;
		                                            var cartErr = parseFloat(cartWeightErrorInput.text.replace(",", ".")) || 0.0;
		                                            relE += Math.max(0.0, cartErr) * 8.0;
		                                            var mlCap = parseFloat(inkCapacityInput.text.replace(",", ".")) || 0.0;
	                                            if (mlCap > 70.0) mlCap = 70.0;
	                                            if (scanCode !== "") {
	                                                hpController.publishString("ink_batch_code", JSON.stringify({
	                                                    "value": scanCode,
	                                                    "operator": "QML"
	                                                }));
	                                            }
	                                            scaleController.confirmTarget(inkN, inkD, cartN, cartD, relE, mlCap);
	                                        }
	                                    }
                                }
                                MotionButton {
                                    id: clearSelectionBtn
                                    opacity: down ? 0.8 : 1.0
                                    text: "CLEAR SELECTION"
                                    Layout.preferredHeight: 50; Layout.preferredWidth: 160
                                    background: Rectangle {
                                        border.color: cPauseBorder
                                        border.width: 1
                                        radius: 5
                                        gradient: Gradient {
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: clearSelectionBtn.down ? cPausePressStart : (clearSelectionBtn.hovered ? cPauseHoverStart : cPauseStart) }
                                            GradientStop { position: 1.0; color: clearSelectionBtn.down ? cPausePressEnd : (clearSelectionBtn.hovered ? cPauseHoverEnd : cPauseEnd) }
                                        }
                                    }
                                    contentItem: Item {
                                        Row {
                                            anchors.centerIn: parent
                                            spacing: 7
                                            Image {
                                                source: "icons/brush_cleaning_white.svg"
                                                width: 24; height: 24
                                                sourceSize.width: 96
                                                sourceSize.height: 96
                                                fillMode: Image.PreserveAspectFit
                                                smooth: true
                                                mipmap: true
                                                antialiasing: true
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                            Text {
                                                text: "CLEAR SELECTION"
	                                                font.pixelSize: tableFont
                                                font.bold: true
                                                color: "#ffffff"
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                        }
                                    }
	                                    onClicked: {
	                                        inkSelector.currentIndex = -1;
	                                        cartSelector.currentIndex = -1;
	                                    }
	                                }
                            }
                        }
                    }

                    // --- ROW 3: TARGET CALCULATIONS ---
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 15

                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: cCard; border.color: cFrameBorder; border.width: 1; radius: 6
                            Column { anchors.centerIn: parent; spacing: 2
	                                Text { text: "TOTAL BATCH (g)"; color: cSubText; font.pixelSize: tableFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
	                                Text { text: scaleController.totalBatchWeight.toFixed(2); color: cAccent; font.pixelSize: valueFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: cCard; border.color: cFrameBorder; border.width: 1; radius: 6
                            Row { anchors.centerIn: parent; spacing: 8
                                Image {
                                    source: "icons/weight_tilde_yellow.svg"
                                    width: 28; height: 28
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    antialiasing: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Column { spacing: 2; anchors.verticalCenter: parent.verticalCenter
	                                Text { text: "MIN WEIGHT (g)"; color: cWarning; font.pixelSize: tableFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
	                                Text { text: scaleController.minWeight.toFixed(2); color: cWarning; font.pixelSize: valueFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                }
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: cCard; border.color: cFrameBorder; border.width: 1; radius: 6
                            Row { anchors.centerIn: parent; spacing: 8
                                Image {
                                    source: "icons/weight_tilde.svg"
                                    width: 28; height: 28
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    antialiasing: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Column { spacing: 2; anchors.verticalCenter: parent.verticalCenter
	                                Text { text: "MAX WEIGHT (g)"; color: cSuccess; font.pixelSize: tableFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
	                                Text { text: scaleController.maxWeight.toFixed(2); color: cSuccess; font.pixelSize: valueFont; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                }
                            }
                        }
                    }

	                }
	            }
	        }
    }    // ── Persistent banner: hiện khi operator chọn NO ở zero-drift popup ──
    // Set bởi scaleController.dismissZeroDrift(); clear khi tare() đc gọi.
    Rectangle {
        id: zeroDriftBanner
        visible: scaleController.zeroDriftPending
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 6
        width: Math.min(parent.width - 40, 720); height: 38
        radius: 6
        color: cAccent
        border.color: cAccent; border.width: 1
        z: 200
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 12; anchors.rightMargin: 8
            spacing: 10
            Text {
                text: "⚠  TEMPO NOT YET TARED — press TARE to clear drift"
	                color: "#0c1726"; font.pixelSize: buttonFont; font.bold: true
                Layout.fillWidth: true
                elide: Text.ElideRight
            }
            MotionButton {
                id: driftTareBtn
                text: "TARE"
                Layout.preferredWidth: 80; Layout.preferredHeight: 28
	                font.pixelSize: tableFont; font.bold: true
                background: Rectangle { color: cField; border.color: cFieldBorder; border.width: 1; radius: 4 }
                contentItem: Text {
                    text: parent.text; color: cAccent
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: scaleController.tare()
            }
        }
    }

    // Popups
    Popup {
        id: overloadPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: 500; height: 200
        modal: true
        closePolicy: Popup.NoAutoClose
        background: Rectangle { color: cCard; border.color: cDanger; border.width: 1; radius: 10 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20
	            Text { text: "WARNING: OVERLOAD!"; color: cDanger; font.pixelSize: titleFont; font.bold: true; Layout.alignment: Qt.AlignHCenter }
	            Text { text: "Scale load exceeds maximum limit. Check immediately."; color: cSubText; font.pixelSize: buttonFont; Layout.alignment: Qt.AlignHCenter }
            Item { Layout.fillHeight: true }
            MotionButton {
                id: ackOverloadBtn
                opacity: down ? 0.8 : 1.0
                text: "ACKNOWLEDGE"
                Layout.alignment: Qt.AlignHCenter
	                font.pixelSize: buttonFont; font.bold: true
                onClicked: { scaleController.ackOverload(); overloadPopup.close() }
                background: Rectangle {
                    radius: 6
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: ackOverloadBtn.down ? cDanger : (ackOverloadBtn.hovered ? "#f0735c" : cDanger) }
                        GradientStop { position: 1.0; color: ackOverloadBtn.down ? "#b53527" : (ackOverloadBtn.hovered ? "#d2473a" : "#b53527") }
                    }
                }
                contentItem: Text { text: parent.text; color: "#ffffff"; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            }
        }
    }

    Popup {
        id: zeroDriftPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: 500; height: 200
        modal: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle { color: cCard; border.color: cFrameBorder; border.width: 1; radius: 10 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20
	            Text { text: "Zero Drift Warning"; color: cAccent; font.pixelSize: titleFont; font.bold: true; Layout.alignment: Qt.AlignHCenter }
	            Text { text: "Loadcell zero drift detected. Re-tare recommended."; color: cSubText; font.pixelSize: buttonFont; Layout.alignment: Qt.AlignHCenter }
            Item { Layout.fillHeight: true }
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 20
                MotionButton {
                    id: zeroDriftTareBtn
                    opacity: down ? 0.8 : 1.0
                    text: "TARE NOW"
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 45
	                    font.pixelSize: buttonFont; font.bold: true
                    onClicked: { scaleController.tare(); zeroDriftPopup.close() }
                    background: Rectangle {
                        radius: 6
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: zeroDriftTareBtn.down ? cActionPressStart : (zeroDriftTareBtn.hovered ? cActionHoverStart : cActionStart) }
                            GradientStop { position: 1.0; color: zeroDriftTareBtn.down ? cActionPressEnd : (zeroDriftTareBtn.hovered ? cActionHoverEnd : cActionEnd) }
                        }
                    }
                    contentItem: Text { text: parent.text; color: "#ffffff"; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                MotionButton {
                    opacity: down ? 0.8 : 1.0
                    text: "NO"
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 45
	                    font.pixelSize: buttonFont; font.bold: true
                    onClicked: { scaleController.dismissZeroDrift(); zeroDriftPopup.close() }
                    background: Rectangle { color: Qt.rgba(0.94, 0.27, 0.27, 0.15); border.color: cDanger; border.width: 1; radius: 6 }
                    contentItem: Text { text: parent.text; color: cDanger; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }
    }

    Connections {
        target: scaleController
        function onOverloadAlarm() { overloadPopup.open() }
        function onZeroDriftAlarm() { zeroDriftPopup.open() }
    }

    // ══════════ VIRTUAL NUMERIC KEYPAD ══════════
    Popup {
        id: numpadPopup
        anchors.centerIn: parent
        width: 320; height: 420
        modal: true
        closePolicy: Popup.CloseOnEscape
        property string currentValue: "0.0"
        property bool isNewEntry: true
        onOpened: isNewEntry = true
        background: Rectangle { color: cCard; radius: 12; border.color: cFrameBorder; border.width: 1 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 15; spacing: 8
	            Text { text: "ENTER VALUE (g)"; color: cAccent; font.pixelSize: buttonFont; font.bold: true; Layout.alignment: Qt.AlignHCenter }
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: 50; radius: 6
                color: cField; border.color: cFieldBorder; border.width: 1
                Text {
                    id: numpadDisplay
                    anchors.centerIn: parent
                    text: numpadPopup.currentValue
	                    color: cAccent; font.pixelSize: valueFont + 4; font.bold: true
                }
            }
            GridLayout {
                columns: 3; rowSpacing: 6; columnSpacing: 6
                Layout.fillWidth: true; Layout.fillHeight: true
                Repeater {
                    model: ["7","8","9","4","5","6","1","2","3",".","0","⌫"]
                    Rectangle {
                        Layout.fillWidth: true; Layout.fillHeight: true; radius: 6
                        color: numBtnMA.pressed ? cAccent : cField
                        border.color: cFrameBorder; border.width: 1
                        Text {
                            anchors.centerIn: parent
	                            text: modelData; color: "#fff"; font.pixelSize: valueFont; font.bold: true
                        }
                        MotionMouseArea {
                            id: numBtnMA; anchors.fill: parent
                            onClicked: {
                                if (modelData === "⌫") {
                                    numpadPopup.isNewEntry = false;
                                    numpadPopup.currentValue = numpadPopup.currentValue.length > 1 ? numpadPopup.currentValue.slice(0, -1) : "0";
                                } else {
                                    var nextVal = numpadPopup.currentValue;
                                    if (numpadPopup.isNewEntry) {
                                        nextVal = (modelData === ".") ? "0." : modelData;
                                        numpadPopup.isNewEntry = false;
                                    } else {
                                        if (modelData === ".") {
                                            if (nextVal.indexOf(".") < 0) {
                                                nextVal += ".";
                                            }
                                        } else {
                                            if (nextVal === "0" || nextVal === "0.0") {
                                                nextVal = modelData;
                                            } else {
                                                nextVal += modelData;
                                            }
                                        }
                                    }
                                    if (nextVal.length > 4) {
                                        nextVal = nextVal.slice(0, 4);
                                    }
                                    numpadPopup.currentValue = nextVal;
                                }
                            }
                        }
                    }
                }
            }
            RowLayout {
                Layout.fillWidth: true; spacing: 10
                MotionButton {
                    opacity: down ? 0.8 : 1.0
                    text: "CANCEL"; Layout.fillWidth: true; Layout.preferredHeight: 44
	                    font.pixelSize: labelFont; font.bold: true
                    onClicked: numpadPopup.close()
                    background: Rectangle { radius: 6; color: Qt.rgba(0.94, 0.27, 0.27, 0.15); border.color: cDanger }
                    contentItem: Text { text: parent.text; font: parent.font; color: cDanger; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                MotionButton {
                    id: numpadOkBtn
                    opacity: down ? 0.8 : 1.0
                    text: "OK"; Layout.fillWidth: true; Layout.preferredHeight: 44
	                    font.pixelSize: labelFont; font.bold: true
                    onClicked: {
                        if (inkTab.numpadTarget) {
                            inkTab.numpadTarget.text = numpadPopup.currentValue;
                        }
                        numpadPopup.close();
                    }
                    background: Rectangle {
                        radius: 6
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: numpadOkBtn.down ? cActionPressStart : (numpadOkBtn.hovered ? cActionHoverStart : cActionStart) }
                            GradientStop { position: 1.0; color: numpadOkBtn.down ? cActionPressEnd : (numpadOkBtn.hovered ? cActionHoverEnd : cActionEnd) }
                        }
                    }
                    contentItem: Text { text: parent.text; font: parent.font; color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }
    }
}
