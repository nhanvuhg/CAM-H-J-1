import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Item {
    id: inkPageRoot

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

    ListModel { id: inkModel }
    ListModel { id: cartModel }
    
    Connections {
        target: scaleController
        function onProfilesChanged() { loadProfiles() }
    }

    property var numpadTarget: null

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // ── Title Bar ──────────────────────────────────────────
        Item {
            Layout.fillWidth: true
            height: 80

            Rectangle {
                anchors.fill: parent
                color: "transparent"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10

                    MotionButton {
    opacity: down ? 0.8 : 1.0
                        text: "◂   BACK"
                        Layout.preferredHeight: 50
                        Layout.preferredWidth: 120
                        font.pixelSize: 18; font.bold: true
                        onClicked: stackView.pop()
                        background: Rectangle {
                            radius: 6
                            color: "#163a52"
                        }
                        contentItem: Text {
                            text: parent.text; font: parent.font
                            color: "#fff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                        }
                    }

                    Item {
                        Layout.fillWidth: true
                        Text {
                            anchors.centerIn: parent
                            text: "LOADCELL & INK SYSTEM"
                            font.pixelSize: 24; font.bold: true; color: "#67d0ff"
                        }
                    }
                    
                    Item { Layout.preferredWidth: 90 }
                }
            }
        }

        // ── Main Content ───────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.margins: 15
            spacing: 15

            // ════════ LEFT PANEL: Monitor & Calibration ════════
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredWidth: 600
                Layout.fillHeight: true
                color: "#081627"
                border.color: "#163a52"
                border.width: 2
                radius: 8

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 12

                    Text { text: "LIVE WEIGHT DISPLAY"; color: "#67d0ff"; font.pixelSize: 22; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                    
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#163a52" }
                    
                    RowLayout {
                        Layout.alignment: Qt.AlignHCenter
                        spacing: 20
                        Text {
                            text: "● LOADCELL: " + scaleController.loadcellStatus
                            color: (scaleController.loadcellStatus == "OK" || scaleController.loadcellStatus == "SIM") ? "#3ed0b4" : "#f0735c"
                            font.pixelSize: 16
                            font.bold: true
                        }
                        Text {
                            text: "Scale node: ● " + (scaleController.scaleNodeConnected ? "CONNECTED" : "DISCONNECTED")
                            color: scaleController.scaleNodeConnected ? "#3ed0b4" : "#f0735c"
                            font.pixelSize: 16
                            font.bold: true
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 110
                        color: "#04080f"
                        border.color: "#36b6ff"
                        border.width: 2
                        radius: 8
                        
                        Text {
                            anchors.centerIn: parent
                            text: scaleController.currentWeight.toFixed(1) + " g"
                            color: "#fff"
                            font.pixelSize: 64
                            font.bold: true
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        
                        Text { text: "Status:"; color: "#9fb3c8"; font.pixelSize: 16 }
                        Rectangle {
                            width: 140; height: 35; radius: 6
                            color: {
                                var s = scaleController.monitorStatus;
                                if(s === "NO_SIGNAL") return "#74899f";
                                if(s === "MEASURING") return "#f5a623";
                                if(s === "PASS") return "#3ed0b4";
                                if(s === "FAIL") return "#f0735c";
                                return "#74899f";
                            }
                            Text { anchors.centerIn: parent; text: scaleController.monitorStatus; color: "#fff"; font.pixelSize: 16; font.bold: true }
                        }
                        
                        Item { Layout.fillWidth: true }
                        
                        Text { text: "Profile:"; color: "#9fb3c8"; font.pixelSize: 16 }
                        Text { text: scaleController.activeProfile === "" ? "NOT SELECTED" : scaleController.activeProfile; color: "#67d0ff"; font.pixelSize: 18; font.bold: true }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 50
                        Layout.maximumHeight: 50
                        spacing: 15
                        
                        MotionButton {
    opacity: down ? 0.8 : 1.0
                            text: "TARE"
                            Layout.fillWidth: true; Layout.preferredHeight: 50; Layout.maximumHeight: 50
                            font.pixelSize: 16; font.bold: true
                            onClicked: scaleController.tare()
                            background: Rectangle { radius: 6; color: "#36b6ff" }
                            contentItem: Text {
                                text: parent.text
                                color: "#ffffff"
                                font: parent.font
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }
                        }
                        MotionButton {
    opacity: down ? 0.8 : 1.0
                            text: "RESET TARE"
                            Layout.fillWidth: true; Layout.preferredHeight: 50; Layout.maximumHeight: 50
                            font.pixelSize: 16; font.bold: true
                            onClicked: scaleController.resetTare()
                            background: Rectangle { radius: 6; color: "#f5a623" }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#163a52" }
                    
                    // ── MOVED: CALIBRATE SCALE ──
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "CALIBRATE SCALE"; color: "#67d0ff"; font.pixelSize: 18; font.bold: true }
                        Item { Layout.fillWidth: true }
                        Text { text: "Status: " + scaleController.calStatus; color: "#f5a623"; font.pixelSize: 14; font.bold: true }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 70
                        color: "#081627"
                        border.color: "#163a52"
                        radius: 6
                        RowLayout {
                            anchors.fill: parent; anchors.margins: 10
                            ColumnLayout {
                                Text { text: "STEP 1 — Empty Scale"; color: "#ecc45a"; font.pixelSize: 14; font.bold: true }
                                Text { text: "Ensure NOTHING is on the scale."; color: "#c7dcef"; font.pixelSize: 12 }
                            }
                            Item { Layout.fillWidth: true }
                            MotionButton {
                                id: setZeroBtn
    opacity: down ? 0.8 : 1.0
                                text: "SET ZERO"
                                Layout.preferredWidth: 120; Layout.preferredHeight: 35
                                font.pixelSize: 14; font.bold: true
                                onClicked: scaleController.startCalibration()
                                background: Rectangle { radius: 4; color: "#36b6ff" }
                                contentItem: Text {
                                    text: setZeroBtn.text; font: setZeroBtn.font
                                    color: "#000000"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                }
                                enabled: scaleController.calStatus === "IDLE" || scaleController.calStatus === "ERROR"
                            }
                        }
                    }



                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 130
                        color: "#081627"
                        border.color: calActive ? "#3ed0b4" : "#163a52"
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
                                        font.pixelSize: 13; font.bold: true
                                    }
                                    Text {
                                        text: {
                                            if (scaleController.calStatus === "WAITING_WEIGHT") return "Suggested: 100g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_2/5") return "Suggested: 250g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_3/5") return "Suggested: 500g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_4/5") return "Suggested: 1000g";
                                            return "Enter known weight below";
                                        }
                                        color: "#9fb3c8"; font.pixelSize: 11
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
                                                if (scaleController.calStatus === "DONE") return "#3ed0b4";
                                                if (scaleController.calStatus === "CONTINUE_CAL_2/5" && n <= 1) return "#3ed0b4";
                                                if (scaleController.calStatus === "CONTINUE_CAL_3/5" && n <= 2) return "#3ed0b4";
                                                if (scaleController.calStatus === "CONTINUE_CAL_4/5" && n <= 3) return "#3ed0b4";
                                                var activeIndex = 0;
                                                var s = scaleController.calStatus;
                                                if (s === "WAITING_WEIGHT") activeIndex = 1;
                                                else if (s === "CONTINUE_CAL_2/5") activeIndex = 2;
                                                else if (s === "CONTINUE_CAL_3/5") activeIndex = 3;
                                                else if (s === "CONTINUE_CAL_4/5") activeIndex = 4;
                                                
                                                if ((s === "WAITING_WEIGHT" || s.startsWith("CONTINUE_CAL")) && n === activeIndex) return "#f5a623";
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
                                    placeholderText: "Enter value (g)"; placeholderTextColor: "#9fb3c8"
                                    Layout.fillWidth: true; Layout.preferredHeight: 45
                                    font.pixelSize: 24; font.bold: true; color: "#f5a623"
                                    horizontalAlignment: TextInput.AlignHCenter
                                    verticalAlignment: TextInput.AlignVCenter
                                    validator: DoubleValidator{}
                                    text: scaleController.lastKnownCalWeight.toString()
                                    enabled: calActive
                                    readOnly: true
                                    background: Rectangle { color: "#0c1726"; radius: 6; border.color: "#36b6ff"; border.width: 2 }
                                    MotionMouseArea {
                                        anchors.fill: parent
                                        onClicked: {
                                            inkPageRoot.numpadTarget = tfCalW;
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
                                    font.pixelSize: 18; font.bold: true
                                    onClicked: {
                                        scaleController.setLastKnownCalWeight(parseFloat(tfCalW.text))
                                        scaleController.setKnownCalibration(scaleController.lastKnownCalWeight)
                                    }
                                    background: Rectangle {
                                        radius: 6
                                        color: calActive ? (scaleController.calStatus === "CONTINUE_CAL_4/5" ? "#9b7bff" : "#36b6ff") : "#74899f"
                                    }
                                    contentItem: Text {
                                        text: applyStep2Btn.text; font: applyStep2Btn.font
                                        color: "#000000"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                    }
                                    enabled: calActive
                                }
                            }
                        }
                    }
                    Item { Layout.fillHeight: true } // SPACER TO PUSH BATCH DOWN

                    Rectangle { Layout.fillWidth: true; height: 2; color: "#163a52" }

                    Text { text: "PRODUCTION RESULTS"; color: "#67d0ff"; font.pixelSize: 18; font.bold: true; font.letterSpacing: 2; Layout.alignment: Qt.AlignHCenter }

                    // ── STAT CARDS ROW ──
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        // TOTAL card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
                            color: "#081627"; border.color: "#36b6ff"; border.width: 2
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
                                Text { text: "TOTAL"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                                Text { text: scaleController.totalBatch.toString(); color: "#fff"; font.pixelSize: 40; font.bold: true; font.family: "monospace"; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "batch"; color: "#74899f"; font.pixelSize: 12; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                        // PASS card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
                            color: "#0a2418"; border.color: "#3ed0b4"; border.width: 2
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
                                Text { text: "✓ PASS"; color: "#3ed0b4"; font.pixelSize: 14; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                                Text { text: scaleController.passBatch.toString(); color: "#3ed0b4"; font.pixelSize: 40; font.bold: true; font.family: "monospace"; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "batch"; color: "#0a2e24"; font.pixelSize: 12; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                        // FAIL card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
                            color: "#220c0b"; border.color: "#f0735c"; border.width: 2
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
                                Text { text: "✗ FAIL"; color: "#f5a394"; font.pixelSize: 14; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                                Text { text: scaleController.failBatch.toString(); color: "#f0735c"; font.pixelSize: 40; font.bold: true; font.family: "monospace"; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "batch"; color: "#b53527"; font.pixelSize: 12; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                    }

                    // ── FAIL STREAK + RESET ──
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 48; radius: 6
                            color: scaleController.consecFails >= 3 ? "#220c0b" : "#06101d"
                            border.color: scaleController.consecFails >= 3 ? "#f0735c" : "#14263c"; border.width: 1
                            RowLayout {
                                anchors.centerIn: parent; spacing: 8
                                Text { text: "⚠ CONSECUTIVE FAILS:"; color: "#9fb3c8"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }
                                Text { text: scaleController.consecFails.toString(); color: scaleController.consecFails >= 3 ? "#f0735c" : "#f5a623"; font.pixelSize: 28; font.bold: true; font.family: "monospace" }
                            }
                        }
                        MotionButton {
    opacity: down ? 0.8 : 1.0
                            text: "RESET BATCH"
                            Layout.preferredWidth: 130; Layout.preferredHeight: 48
                            font.pixelSize: 16; font.bold: true
                            onClicked: scaleController.resetBatch()
                            background: Rectangle { radius: 6; color: "#6f4be0" }
                            contentItem: Text {
                                text: parent.text; font: parent.font
                                color: "#fff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
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
                color: "#081627" // Used #081627 to mimic page header
                border.color: "#163a52"
                border.width: 2
                radius: 8

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 15

                    // Title
                    Text { text: "INK PROFILES"; color: "#67d0ff"; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                    
                    // ── MỤC CHỌN MỰC TƯƠNG TỰ 'SELECT MODE' ──
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#14263c"; opacity: 0.5 }
                    
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 20
                        
                        // Cột 1: Chọn Mực
                        ColumnLayout {
                            Layout.fillWidth: true
                            Text { text: "SELECT INK PROFILE"; color: "#67d0ff"; font.pixelSize: 13; font.bold: true; font.letterSpacing: 1 }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 4
                                color: "#06101d"; border.color: "#f5a623"; border.width: 1
                                ComboBox {
                                    id: inkSelector
                                    anchors.fill: parent; anchors.margins: 1
                                    model: inkModel
                                    textRole: "name"
                                    font.pixelSize: 14; font.bold: true
                                    background: Rectangle { color: "transparent" }
                                    contentItem: Text { text: inkSelector.currentIndex >= 0 ? inkSelector.displayText : "-- Select Ink --"; font: inkSelector.font; color: "#f5a623"; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter }
                                    popup: Popup {
                                        y: inkSelector.height; width: inkSelector.width; implicitHeight: contentItem.implicitHeight + 36; padding: 0
                                        background: Rectangle { color: "#06101d"; border.color: "#f5a623"; border.width: 1; radius: 4 }
                                        contentItem: Column {
                                            width: parent.width
                                            // Table Header
                                            Rectangle {
                                                width: parent.width; height: 30; color: "#0c1726"
                                                Row {
                                                    anchors.fill: parent; spacing: 0
                                                    Item { width: parent.width * 0.12; height: parent.height; Text { anchors.centerIn: parent; text: "No"; color: "#67d0ff"; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: "#14263c" }
                                                    Item { width: parent.width * 0.48 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "INK NAME"; color: "#67d0ff"; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: "#14263c" }
                                                    Item { width: parent.width * 0.28 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "CART WEIGHT"; color: "#67d0ff"; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: "#14263c" }
                                                    Item { width: parent.width * 0.12 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DEL"; color: "#f0735c"; font.pixelSize: 11; font.bold: true } }
                                                }
                                            }
                                            Rectangle { width: parent.width; height: 2; color: "#f5a623" }
                                            // Table Rows
                                            ListView {
                                                width: parent.width; implicitHeight: contentHeight; clip: true
                                                model: inkSelector.delegateModel
                                            }
                                        }
                                    }
                                    delegate: ItemDelegate {
                                        width: inkSelector.width; height: 34
                                        contentItem: Item {
                                            anchors.fill: parent
                                            Row {
                                                anchors.fill: parent; spacing: 0
                                                Item { width: parent.width * 0.12; height: parent.height; Text { anchors.centerIn: parent; text: (index+1).toString(); color: inkSelector.highlightedIndex === index ? "#000" : "#9fb3c8"; font.pixelSize: 13; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: "#0c1726" }
                                                Item { width: parent.width * 0.48 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: model.name; color: inkSelector.highlightedIndex === index ? "#000" : "#f5a623"; font.pixelSize: 13; font.bold: true; font.family: "monospace"; elide: Text.ElideRight; width: parent.width - 10 } }
                                                Rectangle { width: 1; height: parent.height; color: "#0c1726" }
                                                Item { width: parent.width * 0.28 - 1; height: parent.height; Text { anchors.centerIn: parent; text: model.density.toFixed(2) + " g"; color: inkSelector.highlightedIndex === index ? "#000" : "#f5a623"; font.pixelSize: 13; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: "#0c1726" }
                                                Item {
                                                    width: parent.width * 0.12 - 1; height: parent.height
                                                    Rectangle {
                                                        anchors.centerIn: parent; width: 22; height: 22; radius: 4; color: delInkMA.pressed ? "#b53527" : "#f0735c"
                                                        Text { anchors.centerIn: parent; text: "✕"; font.bold: true; font.pixelSize: 12; color: "#fff" }
                                                        MotionMouseArea { id: delInkMA; anchors.fill: parent; onClicked: { scaleController.deleteInkProfile(model.name); } }
                                                    }
                                                }
                                            }
                                            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: "#0c1726" }
                                        }
                                        background: Rectangle { color: inkSelector.highlightedIndex === index ? "#f5a623" : "#06101d" }
                                    }
                                }
                            }
                        }

                        // Cột 2: Chọn Vỏ
                        ColumnLayout {
                            Layout.fillWidth: true
                            Text { text: "SELECT CARTRIDGE TYPE"; color: "#67d0ff"; font.pixelSize: 13; font.bold: true; font.letterSpacing: 1 }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 4
                                color: "#06101d"; border.color: "#f5a623"; border.width: 1
                                ComboBox {
                                    id: cartSelector
                                    anchors.fill: parent; anchors.margins: 1
                                    model: cartModel
                                    textRole: "name"
                                    font.pixelSize: 14; font.bold: true
                                    background: Rectangle { color: "transparent" }
                                    contentItem: Text { text: cartSelector.currentIndex >= 0 ? cartSelector.displayText : "-- Select Cartridge --"; font: cartSelector.font; color: "#f5a623"; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter }
                                    popup: Popup {
                                        y: cartSelector.height; width: cartSelector.width; implicitHeight: contentItem.implicitHeight + 36; padding: 0
                                        background: Rectangle { color: "#06101d"; border.color: "#f5a623"; border.width: 1; radius: 4 }
                                        contentItem: Column {
                                            width: parent.width
                                            Rectangle {
                                                width: parent.width; height: 30; color: "#0c1726"
                                                Row {
                                                    anchors.fill: parent; spacing: 0
                                                    Item { width: parent.width * 0.12; height: parent.height; Text { anchors.centerIn: parent; text: "No"; color: "#67d0ff"; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: "#14263c" }
                                                    Item { width: parent.width * 0.48 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "CART NAME"; color: "#67d0ff"; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: "#14263c" }
                                                    Item { width: parent.width * 0.28 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DENSITY"; color: "#67d0ff"; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: "#14263c" }
                                                    Item { width: parent.width * 0.12 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DEL"; color: "#f0735c"; font.pixelSize: 11; font.bold: true } }
                                                }
                                            }
                                            Rectangle { width: parent.width; height: 2; color: "#f5a623" }
                                            ListView {
                                                width: parent.width; implicitHeight: contentHeight; clip: true
                                                model: cartSelector.delegateModel
                                            }
                                        }
                                    }
                                    delegate: ItemDelegate {
                                        width: cartSelector.width; height: 34
                                        contentItem: Item {
                                            anchors.fill: parent
                                            Row {
                                                anchors.fill: parent; spacing: 0
                                                Item { width: parent.width * 0.12; height: parent.height; Text { anchors.centerIn: parent; text: (index+1).toString(); color: cartSelector.highlightedIndex === index ? "#000" : "#9fb3c8"; font.pixelSize: 13; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: "#0c1726" }
                                                Item { width: parent.width * 0.48 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: model.name; color: cartSelector.highlightedIndex === index ? "#000" : "#f5a623"; font.pixelSize: 13; font.bold: true; font.family: "monospace"; elide: Text.ElideRight; width: parent.width - 10 } }
                                                Rectangle { width: 1; height: parent.height; color: "#0c1726" }
                                                Item { width: parent.width * 0.28 - 1; height: parent.height; Text { anchors.centerIn: parent; text: model.density.toFixed(2) + " g"; color: cartSelector.highlightedIndex === index ? "#000" : "#f5a623"; font.pixelSize: 13; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: "#0c1726" }
                                                Item {
                                                    width: parent.width * 0.12 - 1; height: parent.height
                                                    Rectangle {
                                                        anchors.centerIn: parent; width: 22; height: 22; radius: 4; color: delCartMA.pressed ? "#b53527" : "#f0735c"
                                                        Text { anchors.centerIn: parent; text: "✕"; font.bold: true; font.pixelSize: 12; color: "#fff" }
                                                        MotionMouseArea { id: delCartMA; anchors.fill: parent; onClicked: { scaleController.deleteCartProfile(model.name); } }
                                                    }
                                                }
                                            }
                                            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: "#0c1726" }
                                        }
                                        background: Rectangle { color: cartSelector.highlightedIndex === index ? "#f5a623" : "#06101d" }
                                    }
                                }
                            }
                        }
                    }

                    // --- DETAILS AREA ---
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 190
                        color: "#06101d"; border.color: "#36b6ff"; border.width: 1; radius: 6
                        
                        GridLayout {
                            anchors.fill: parent; anchors.margins: 15
                            columns: 3; rowSpacing: 15; columnSpacing: 30

                            // Row 1
                            RowLayout {
                                Text { text: "INK NAME:"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true }
                                Text { text: inkSelector.currentIndex >= 0 ? inkModel.get(inkSelector.currentIndex).name : "--"; color: "#f5a623"; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "DENSITY:"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true }
                                Text { text: inkSelector.currentIndex >= 0 ? inkModel.get(inkSelector.currentIndex).density.toFixed(2) + " g" : "--"; color: "#f5a623"; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "RELATIVE ERROR (g):"; color: "#67d0ff"; font.pixelSize: 14; font.bold: true }
                                Rectangle {
                                    width: 70; height: 35; color: "#06101d"; border.color: "#67d0ff"; border.width: 1; radius: 4
                                    TextInput {
                                        id: relativeErrorInput
                                        anchors.fill: parent; anchors.margins: 2; color: "#f5a623"; font.pixelSize: 16; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.family: "monospace"
                                        text: "1.0"
                                        readOnly: true
                                        MotionMouseArea { anchors.fill: parent; onClicked: { inkPageRoot.numpadTarget = relativeErrorInput; numpadPopup.currentValue = relativeErrorInput.text; numpadPopup.open() } }
                                    }
                                }
                            }

                            // Row 2
                            RowLayout {
                                Text { text: "CARTRIDGE TYPE:"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true }
                                Text { text: cartSelector.currentIndex >= 0 ? cartModel.get(cartSelector.currentIndex).name : "--"; color: "#f5a623"; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "CART WEIGHT:"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true }
                                Text { text: cartSelector.currentIndex >= 0 ? cartModel.get(cartSelector.currentIndex).density.toFixed(2) + " g" : "--"; color: "#f5a623"; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "ML FILL (ml):"; color: "#67d0ff"; font.pixelSize: 14; font.bold: true }
                                Rectangle {
                                    width: 70; height: 35; color: "#06101d"; border.color: "#67d0ff"; border.width: 1; radius: 4
                                    TextInput {
                                        id: inkCapacityInput
                                        anchors.fill: parent; anchors.margins: 2; color: "#f5a623"; font.pixelSize: 16; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.family: "monospace"
                                        text: scaleController.inkCapacity.toString()
                                        readOnly: true
                                        MotionMouseArea { anchors.fill: parent; onClicked: { inkPageRoot.numpadTarget = inkCapacityInput; numpadPopup.currentValue = inkCapacityInput.text; numpadPopup.open() } }
                                        Connections {
                                            target: scaleController
                                            function onInkCapacityChanged() {
                                                if (inkPageRoot.numpadTarget !== inkCapacityInput) {
                                                    inkCapacityInput.text = scaleController.inkCapacity.toString();
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


                            // Row 3: Current ML Fill
                            RowLayout {
                                Text { text: "CURRENT ML FILL:"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true }
                                Rectangle {
                                    width: 80; height: 35; color: "#06101d"; border.color: "#36b6ff"; border.width: 1; radius: 4
                                    Text {
                                        anchors.centerIn: parent; text: scaleController.currentMlFill.toFixed(1) + " ml"; color: "#3ed0b4"; font.pixelSize: 16; font.bold: true; font.family: "monospace"
                                    }
                                }
                            }

                            // Row 4: CONFIRM BTN & CLEAR SELECTION
                            RowLayout {
                                Layout.columnSpan: 2; Layout.fillWidth: true; spacing: 15
                                Item { Layout.fillWidth: true }
                                MotionButton {
    opacity: down ? 0.8 : 1.0
                                    text: "APPLY TARGET (RUN)"
                                    Layout.preferredHeight: 50; Layout.preferredWidth: 240
                                    background: Rectangle { color: "#36b6ff"; radius: 5 }
                                    contentItem: Text { text: parent.text; font.pixelSize: 16; font.bold: true; color: "#000"; horizontalAlignment: Qt.AlignHCenter; verticalAlignment: Qt.AlignVCenter }
                                    onClicked: {
                                        if (inkSelector.currentIndex >= 0 && cartSelector.currentIndex >= 0) {
                                            var inkN = inkModel.get(inkSelector.currentIndex).name;
                                            var inkD = inkModel.get(inkSelector.currentIndex).density;
                                            var cartN = cartModel.get(cartSelector.currentIndex).name;
                                            var cartD = cartModel.get(cartSelector.currentIndex).density;
                                            var relE = parseFloat(relativeErrorInput.text.replace(",", ".")) || 0.0;
                                            var mlCap = parseFloat(inkCapacityInput.text.replace(",", ".")) || 0.0;
                                            if (mlCap > 70.0) mlCap = 70.0;
                                            scaleController.confirmTarget(inkN, inkD, cartN, cartD, relE, mlCap);
                                        }
                                    }
                                }
                                MotionButton {
    opacity: down ? 0.8 : 1.0
                                    text: "CLEAR SELECTION"
                                    Layout.preferredHeight: 50; Layout.preferredWidth: 160
                                    background: Rectangle { color: "#2c0c0b"; border.color: "#f0735c"; border.width: 1; radius: 5 }
                                    contentItem: Text { text: parent.text; font.pixelSize: 13; font.bold: true; color: "#f0735c"; horizontalAlignment: Qt.AlignHCenter; verticalAlignment: Qt.AlignVCenter }
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
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: "#04080f"; border.color: "#163a52"; border.width: 1; radius: 6
                            Column { anchors.centerIn: parent; spacing: 2
                                Text { text: "TOTAL BATCH (g)"; color: "#9fb3c8"; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: scaleController.totalBatchWeight.toFixed(2); color: "#1f86e0"; font.pixelSize: 22; font.bold: true; font.family: "monospace"; anchors.horizontalCenter: parent.horizontalCenter }
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: "#04080f"; border.color: "#f5a623"; border.width: 1; radius: 6
                            Column { anchors.centerIn: parent; spacing: 2
                                Text { text: "MIN WEIGHT (g)"; color: "#f5a623"; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: scaleController.minWeight.toFixed(2); color: "#f5a623"; font.pixelSize: 22; font.bold: true; font.family: "monospace"; anchors.horizontalCenter: parent.horizontalCenter }
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: "#04080f"; border.color: "#3ed0b4"; border.width: 1; radius: 6
                            Column { anchors.centerIn: parent; spacing: 2
                                Text { text: "MAX WEIGHT (g)"; color: "#3ed0b4"; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: scaleController.maxWeight.toFixed(2); color: "#3ed0b4"; font.pixelSize: 22; font.bold: true; font.family: "monospace"; anchors.horizontalCenter: parent.horizontalCenter }
                            }
                        }
                    }

                    Item { Layout.fillHeight: true } // spacer

                    // --- ROW 4: CREATE PROFILE ---
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#14263c"; opacity: 0.5 }
                    Text { text: "CREATE NEW PROFILE"; color: "#67d0ff"; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 20

                        // Create Ink
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 180; color: "transparent"; border.color: "#163a52"; border.width: 2; radius: 6
                            GridLayout {
                                anchors.fill: parent; anchors.margins: 15; columns: 2; columnSpacing: 15; rowSpacing: 15
                                Text { text: "INK NAME"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "CART WEIGHT (g)"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                                
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: 40; color: "#06101d"; border.color: "#67d0ff"; border.width: 2; radius: 4
                                    TextInput { id: newInkName; anchors.fill: parent; anchors.margins: 4; color: "#fff"; font.pixelSize: 18; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; clip: true }
                                    Text { anchors.centerIn: parent; text: "--Type Ink Name--"; color: "#74899f"; font.pixelSize: 15; font.italic: true; visible: newInkName.text.length === 0 && !newInkName.activeFocus }
                                }
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: 40; color: "#06101d"; border.color: "#67d0ff"; border.width: 2; radius: 4
                                    TextInput { id: newInkDensity; anchors.fill: parent; anchors.margins: 4; color: "#f5a623"; font.pixelSize: 18; font.bold:true; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; readOnly: true; text: "0.0"; MotionMouseArea { anchors.fill: parent; onClicked: { inkPageRoot.numpadTarget = newInkDensity; numpadPopup.currentValue = newInkDensity.text; numpadPopup.open() } } }
                                }
                                
                                MotionButton {
    opacity: down ? 0.8 : 1.0
                                    text: "SAVE INK"
                                    Layout.preferredHeight: 45; Layout.fillWidth: true; Layout.columnSpan: 2; background: Rectangle { color: "#04080f"; border.color: "#67d0ff"; border.width: 2; radius: 6 }
                                    contentItem: Text { text: parent.text; color: "#67d0ff"; font.pixelSize: 16; font.bold: true; horizontalAlignment: Qt.AlignHCenter; verticalAlignment: Qt.AlignVCenter }
                                    onClicked: {
                                        var d = parseFloat(newInkDensity.text.replace(",", "."));
                                        if (d > 0 && newInkName.text.trim() !== "") {
                                            scaleController.createInkProfile(newInkName.text.trim(), d);
                                            newInkName.text = ""; newInkDensity.text = "0.0";
                                        }
                                    }
                                }
                            }
                        }

                        // Create Cart
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 180; color: "transparent"; border.color: "#163a52"; border.width: 2; radius: 6
                            GridLayout {
                                anchors.fill: parent; anchors.margins: 15; columns: 2; columnSpacing: 15; rowSpacing: 15
                                Text { text: "CART NAME"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "DENSITY (g)"; color: "#9fb3c8"; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                                
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: 40; color: "#06101d"; border.color: "#67d0ff"; border.width: 2; radius: 4
                                    TextInput { id: newCartName; anchors.fill: parent; anchors.margins: 4; color: "#fff"; font.pixelSize: 18; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; clip: true }
                                    Text { anchors.centerIn: parent; text: "--Type Cart Name--"; color: "#74899f"; font.pixelSize: 15; font.italic: true; visible: newCartName.text.length === 0 && !newCartName.activeFocus }
                                }
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: 40; color: "#06101d"; border.color: "#67d0ff"; border.width: 2; radius: 4
                                    TextInput { id: newCartDensity; anchors.fill: parent; anchors.margins: 4; color: "#f5a623"; font.pixelSize: 18; font.bold:true; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; readOnly: true; text: "0.0"; MotionMouseArea { anchors.fill: parent; onClicked: { inkPageRoot.numpadTarget = newCartDensity; numpadPopup.currentValue = newCartDensity.text; numpadPopup.open() } } }
                                }
                                
                                MotionButton {
    opacity: down ? 0.8 : 1.0
                                    text: "SAVE CART"
                                    Layout.preferredHeight: 45; Layout.fillWidth: true; Layout.columnSpan: 2; background: Rectangle { color: "#04080f"; border.color: "#67d0ff"; border.width: 2; radius: 6 }
                                    contentItem: Text { text: parent.text; color: "#67d0ff"; font.pixelSize: 16; font.bold: true; horizontalAlignment: Qt.AlignHCenter; verticalAlignment: Qt.AlignVCenter }
                                    onClicked: {
                                        var d = parseFloat(newCartDensity.text.replace(",", "."));
                                        if (d > 0 && newCartName.text.trim() !== "") {
                                            scaleController.createCartProfile(newCartName.text.trim(), d);
                                            newCartName.text = ""; newCartDensity.text = "0.0";
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

    // ── Persistent banner: hiện khi operator chọn NO ở zero-drift popup ──
    // Set bởi scaleController.dismissZeroDrift(); clear khi tare() đc gọi.
    Rectangle {
        id: zeroDriftBanner
        visible: scaleController.zeroDriftPending
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 6
        width: Math.min(parent.width - 40, 720); height: 38
        radius: 6
        color: "#f5a623"
        border.color: "#e2761b"; border.width: 1
        z: 200
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 12; anchors.rightMargin: 8
            spacing: 10
            Text {
                text: "⚠  TEMPO NOT YET TARED — press TARE to clear drift"
                color: "#0c1726"; font.pixelSize: 16; font.bold: true
                Layout.fillWidth: true
                elide: Text.ElideRight
            }
            MotionButton {
                text: "TARE"
                Layout.preferredWidth: 80; Layout.preferredHeight: 28
                font.pixelSize: 13; font.bold: true
                background: Rectangle { color: "#ffffff"; radius: 4 }
                contentItem: Text {
                    text: parent.text; color: "#f5a623"
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
        anchors.centerIn: parent
        width: 500; height: 200
        modal: true
        closePolicy: Popup.NoAutoClose
        background: Rectangle { color: "#f0735c"; radius: 10 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20
            Text { text: "WARNING: OVERLOAD!"; color: "#fff"; font.pixelSize: 26; font.bold: true; Layout.alignment: Qt.AlignHCenter }
            Text { text: "Scale load exceeds maximum limit. Check immediately."; color: "#fff"; font.pixelSize: 18; Layout.alignment: Qt.AlignHCenter }
            Item { Layout.fillHeight: true }
            MotionButton {
    opacity: down ? 0.8 : 1.0
                text: "ACKNOWLEDGE"
                Layout.alignment: Qt.AlignHCenter
                font.pixelSize: 18; font.bold: true
                onClicked: { scaleController.ackOverload(); overloadPopup.close() }
                background: Rectangle { radius: 6; color: "#fff"; border.width: 0 }
                contentItem: Text { text: parent.text; color: "#f0735c"; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            }
        }
    }

    Popup {
        id: zeroDriftPopup
        anchors.centerIn: parent
        width: 500; height: 200
        modal: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle { color: "#f5a623"; radius: 10 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20
            Text { text: "Zero Drift Warning"; color: "#fff"; font.pixelSize: 26; font.bold: true; Layout.alignment: Qt.AlignHCenter }
            Text { text: "Loadcell zero drift detected. Re-tare recommended."; color: "#fff"; font.pixelSize: 18; Layout.alignment: Qt.AlignHCenter }
            Item { Layout.fillHeight: true }
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 20
                MotionButton {
                    opacity: down ? 0.8 : 1.0
                    text: "TARE NOW"
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 45
                    font.pixelSize: 18; font.bold: true
                    onClicked: { scaleController.tare(); zeroDriftPopup.close() }
                    background: Rectangle { radius: 6; color: "#fff"; border.width: 0 }
                    contentItem: Text { text: parent.text; color: "#f5a623"; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                MotionButton {
                    opacity: down ? 0.8 : 1.0
                    text: "NO"
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 45
                    font.pixelSize: 18; font.bold: true
                    onClicked: { scaleController.dismissZeroDrift(); zeroDriftPopup.close() }
                    background: Rectangle { radius: 6; color: "#f0735c"; border.width: 0 }
                    contentItem: Text { text: parent.text; color: "#fff"; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
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
        background: Rectangle { color: "#06101d"; radius: 12; border.color: "#67d0ff"; border.width: 2 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 15; spacing: 8
            Text { text: "ENTER VALUE (g)"; color: "#67d0ff"; font.pixelSize: 16; font.bold: true; Layout.alignment: Qt.AlignHCenter }
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: 50; radius: 6
                color: "#081627"; border.color: "#f5a623"; border.width: 2
                Text {
                    id: numpadDisplay
                    anchors.centerIn: parent
                    text: numpadPopup.currentValue
                    color: "#f5a623"; font.pixelSize: 28; font.bold: true; font.family: "monospace"
                }
            }
            GridLayout {
                columns: 3; rowSpacing: 6; columnSpacing: 6
                Layout.fillWidth: true; Layout.fillHeight: true
                Repeater {
                    model: ["7","8","9","4","5","6","1","2","3",".","0","⌫"]
                    Rectangle {
                        Layout.fillWidth: true; Layout.fillHeight: true; radius: 6
                        color: numBtnMA.pressed ? "#163a52" : "#081627"
                        border.color: "#14263c"; border.width: 1
                        Text {
                            anchors.centerIn: parent
                            text: modelData; color: "#fff"; font.pixelSize: 22; font.bold: true
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
                    font.pixelSize: 14; font.bold: true
                    onClicked: numpadPopup.close()
                    background: Rectangle { radius: 6; color: "#2c0c0b"; border.color: "#f0735c" }
                    contentItem: Text { text: parent.text; font: parent.font; color: "#f0735c"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                MotionButton {
    opacity: down ? 0.8 : 1.0
                    text: "OK"; Layout.fillWidth: true; Layout.preferredHeight: 44
                    font.pixelSize: 14; font.bold: true
                    onClicked: {
                        if (inkPageRoot.numpadTarget) {
                            inkPageRoot.numpadTarget.text = numpadPopup.currentValue;
                        }
                        numpadPopup.close();
                    }
                    background: Rectangle { radius: 6; color: "#36b6ff" }
                    contentItem: Text { text: parent.text; font: parent.font; color: "#000"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }
    }
}
