import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root

    property bool pageActive: visible
    property color panelColor: "#0d1e32"
    property color panelColorDeep: "#06101d"
    property color borderColor: "#163a52"
    property color textColor: "#d6f1ff"
    property color mutedColor: "#9fb3c8"
    property color accentColor: "#7fcdf5"
    property color warningColor: "#f5a623"
    property color errorColor: "#f0735c"
    property color successColor: "#3ed0b4"

    implicitWidth: 50
    implicitHeight: 50

    readonly property int totalCount: systemAlertController.errorCount
                                      + systemAlertController.warningCount
    readonly property bool hasError: systemAlertController.errorCount > 0
    readonly property bool needsAcknowledge:
        systemAlertController.unacknowledgedWarningCount > 0
    readonly property color stateColor: hasError ? errorColor
                                                 : (needsAcknowledge ? warningColor
                                                                     : (totalCount > 0 ? accentColor
                                                                                       : borderColor))

    function openPanel() {
        alertPopup.open()
    }

    function displayLevel(level) {
        var token = String(level || "").trim().toUpperCase()
        switch (token) {
        case "ERROR": return qsTr("ERROR")
        case "WARNING": return qsTr("WARNING")
        case "INFO": return qsTr("INFO")
        default: return level || ""
        }
    }

    function displayArea(area) {
        var token = String(area || "").trim().toUpperCase()
        switch (token) {
        case "ROBOT": return qsTr("ROBOT")
        case "FEEDER": return qsTr("FEEDER")
        case "SCALE": return qsTr("SCALE")
        case "VFD": return qsTr("VFD")
        case "CAMERA": return qsTr("CAMERA")
        case "FILL_HP": return qsTr("FILL_HP")
        default: return area || ""
        }
    }

    Connections {
        target: systemAlertController
        function onAttentionRequested() {
            if (root.pageActive)
                root.openPanel()
        }
    }

    MotionButton {
        id: alertButton
        anchors.fill: parent
        hoverScale: 1.012
        pressScale: 0.99
        onClicked: root.openPanel()

        background: Rectangle {
            radius: 6
            color: alertButton.pressed
                   ? Qt.darker(root.panelColor, 1.12)
                   : (root.hasError
                      ? Qt.rgba(root.errorColor.r, root.errorColor.g, root.errorColor.b, 0.10)
                      : (root.needsAcknowledge
                         ? Qt.rgba(root.warningColor.r, root.warningColor.g,
                                   root.warningColor.b, 0.08)
                         : Qt.rgba(0.03, 0.09, 0.16, 0.62)))
            border.color: root.borderColor
            border.width: 1
            Behavior on color { ColorAnimation { duration: 160 } }
        }

        contentItem: Item {
            Image {
                anchors.centerIn: parent
                source: "icons/message_circle_warning.svg"
                width: 34
                height: 34
                // Bo dung SVG cua Qt (SVG Tiny) rasterize net cong khong deu o
                // kich thuoc nho: vien bong bong bi meo, day mong khong dong nhat.
                // Dung o 4x kich thuoc hien thi roi thu nho — sai so cua bo dung
                // bi trung binh hoa nen vien tro lai lien mach.
                sourceSize.width: 136
                sourceSize.height: 136
                fillMode: Image.PreserveAspectFit
                smooth: true
            }

            Rectangle {
                visible: root.totalCount > 0
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.rightMargin: -3
                anchors.topMargin: -3
                width: 21
                height: 21
                radius: 10.5
                color: root.hasError ? root.errorColor : root.warningColor
                border.color: root.panelColorDeep
                border.width: 2

                Text {
                    anchors.centerIn: parent
                    text: root.totalCount > 9 ? "9+" : root.totalCount
                    color: "#ffffff"
                    font.pixelSize: 10
                    font.bold: true
                }
            }

            HoverHint {
                visible: alertButton.hovered
                label: root.totalCount === 0
                       ? qsTr("System warnings and errors")
                       : (root.hasError
                          ? (systemAlertController.errorCount === 1
                             ? qsTr("1 error blocking START")
                             : qsTr("%1 errors blocking START")
                                 .arg(systemAlertController.errorCount))
                          : (root.needsAcknowledge
                             ? (systemAlertController.unacknowledgedWarningCount === 1
                                ? qsTr("1 warning requires acknowledgement")
                                : qsTr("%1 warnings require acknowledgement")
                                    .arg(systemAlertController.unacknowledgedWarningCount))
                             : (systemAlertController.warningCount === 1
                                ? qsTr("1 warning acknowledged")
                                : qsTr("%1 warnings acknowledged")
                                    .arg(systemAlertController.warningCount))))
                bc: root.stateColor
                tc: root.textColor
            }
        }
    }

    Popup {
        id: alertPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(860, parent.width - 80)
        height: Math.min(720, parent.height - 80)
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        padding: 0

        background: Rectangle {
            radius: 10
            color: "#f20a1424"
            border.color: root.borderColor
            border.width: 1
        }

        contentItem: ColumnLayout {
            spacing: 0

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 82
                color: Qt.rgba(0.04, 0.10, 0.18, 0.90)
                radius: 10

                Rectangle {
                    anchors.bottom: parent.bottom
                    width: parent.width
                    height: 12
                    color: parent.color
                }

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 22
                    anchors.rightMargin: 64
                    spacing: 14

                    Image {
                        source: "icons/message_circle_warning.svg"
                        Layout.preferredWidth: 38
                        Layout.preferredHeight: 38
                        sourceSize.width: 152
                        sourceSize.height: 152
                        fillMode: Image.PreserveAspectFit
                        smooth: true
                    }

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 3

                        Text {
                            text: qsTr("SYSTEM WARNINGS & ERRORS")
                            color: root.textColor
                            font.pixelSize: 20
                            font.bold: true
                            font.letterSpacing: 1.1
                        }
                        Text {
                            text: root.totalCount === 0
                                  ? qsTr("No active warnings or errors")
                                  : (systemAlertController.errorCount === 1
                                     ? qsTr("1 ERROR")
                                     : qsTr("%1 ERRORS").arg(systemAlertController.errorCount))
                                    + "  -  "
                                    + (systemAlertController.warningCount === 1
                                       ? qsTr("1 WARNING")
                                       : qsTr("%1 WARNINGS").arg(systemAlertController.warningCount))
                            color: root.mutedColor
                            font.pixelSize: 13
                        }
                    }

                    MotionButton {
                        visible: systemAlertController.unacknowledgedWarningCount > 1
                        Layout.preferredWidth: 168
                        Layout.preferredHeight: 42
                        text: qsTr("ACK ALL WARNINGS")
                        font.pixelSize: 11
                        font.bold: true
                        onClicked: systemAlertController.acknowledgeAllWarnings()
                        background: Rectangle {
                            radius: 8
                            color: parent.pressed ? Qt.darker(root.warningColor, 1.2) : "#5a370d"
                            border.color: root.warningColor
                        }
                        contentItem: Text {
                            text: parent.text
                            font: parent.font
                            color: "#ffffff"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }

                }

                // Bang canh bao la modal nen moi cham ra ngoai deu bi nuot —
                // nut chup man hinh o header khong bam duoc, va operator khong
                // luu duoc anh canh bao lam tai lieu. Dat nut chup NGAY TRONG
                // bang, canh nut dong.
                MotionButton {
                    id: shotAlertButton
                    anchors.right: closeAlertButton.left
                    anchors.rightMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    width: 44
                    height: 44
                    z: 2
                    onClicked: {
                        if (robotController.captureScreenshot() !== "")
                            shotFlashTimer.restart()
                    }
                    background: Rectangle {
                        radius: 8
                        color: shotAlertButton.pressed ? "#123b52" : "#163a52"
                        // Sang len 2.5s khi anh da ghi xong: bang modal khong co
                        // cho hien thong bao, va im lang la dau hieu chup hong.
                        border.color: shotFlashTimer.running ? "#7fcdf5" : root.errorColor
                        border.width: shotFlashTimer.running ? 2 : 1
                        Behavior on border.color { ColorAnimation { duration: 140 } }
                    }
                    contentItem: Image {
                        source: "qrc:/qml/icons/camera.svg"
                        sourceSize.width: 24; sourceSize.height: 24
                        fillMode: Image.PreserveAspectFit
                        anchors.centerIn: parent
                    }
                    Timer { id: shotFlashTimer; interval: 2500 }
                }

                MotionButton {
                    id: closeAlertButton
                    anchors.right: parent.right
                    // Keep the same inset on the top and right edges.
                    anchors.rightMargin: (parent.height - height) / 2
                    anchors.verticalCenter: parent.verticalCenter
                    width: 44
                    height: 44
                    z: 2
                    text: "×"
                    font.pixelSize: 28
                    onClicked: alertPopup.close()
                    background: Rectangle {
                        radius: 8
                        color: closeAlertButton.pressed ? "#6f2327" : "#8f2f32"
                        border.color: root.errorColor
                    }
                    contentItem: Text {
                        text: closeAlertButton.text
                        font: closeAlertButton.font
                        color: "#ffffff"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }

            Rectangle {
                visible: !systemAlertController.canStart
                Layout.fillWidth: true
                Layout.preferredHeight: 48
                color: root.hasError ? "#361216" : "#35250c"
                border.color: "transparent"
                border.width: 0

                Rectangle {
                    anchors { top: parent.top; bottom: parent.bottom; left: parent.left }
                    width: 4
                    color: root.hasError ? root.errorColor : root.warningColor
                }

                Rectangle {
                    anchors { left: parent.left; right: parent.right; bottom: parent.bottom }
                    height: 1
                    color: root.borderColor
                }

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 20
                    anchors.rightMargin: 20
                    spacing: 10
                    Rectangle {
                        width: 9
                        height: 9
                        radius: 4.5
                        color: root.hasError ? root.errorColor : root.warningColor
                    }
                    Text {
                        Layout.fillWidth: true
                        text: systemAlertController.startBlockReason
                        color: "#ffffff"
                        font.pixelSize: 14
                        font.bold: true
                    }
                }
            }

            Item {
                Layout.fillWidth: true
                Layout.fillHeight: true

                ColumnLayout {
                    visible: root.totalCount === 0
                    anchors.centerIn: parent
                    spacing: 12

                    Rectangle {
                        Layout.alignment: Qt.AlignHCenter
                        width: 58
                        height: 58
                        radius: 29
                        color: "#16352f"
                        border.color: root.successColor
                        Text {
                            anchors.centerIn: parent
                            text: "✓"
                            color: root.successColor
                            font.pixelSize: 30
                            font.bold: true
                        }
                    }
                    Text {
                        Layout.alignment: Qt.AlignHCenter
                        text: qsTr("NO ACTIVE SYSTEM WARNINGS OR ERRORS")
                        color: root.textColor
                        font.pixelSize: 16
                        font.bold: true
                    }
                    Text {
                        Layout.alignment: Qt.AlignHCenter
                        text: qsTr("AUTO/AI error history is still saved in Production Output.")
                        color: root.mutedColor
                        font.pixelSize: 13
                    }
                }

                ListView {
                    id: alertList
                    visible: root.totalCount > 0
                    anchors.fill: parent
                    anchors.margins: 16
                    spacing: 10
                    clip: true
                    boundsBehavior: Flickable.StopAtBounds
                    model: systemAlertController.activeAlerts

                    // AsNeeded chi hien luc dang vuot roi tu mo di. Tren man hinh
                    // cam ung khong co con lan chuot, van hanh khong co cach nao
                    // biet danh sach con loi phia duoi. Giu thanh hien thuong
                    // truc khi tran, va du day de cham duoc bang ngon tay.
                    ScrollBar.vertical: ScrollBar {
                        id: alertScrollBar
                        policy: alertList.contentHeight > alertList.height
                                ? ScrollBar.AlwaysOn : ScrollBar.AlwaysOff
                        width: 12
                        background: Rectangle {
                            radius: 6
                            color: "#0d1522"
                            border.color: root.borderColor
                            border.width: 1
                        }
                        contentItem: Rectangle {
                            implicitWidth: 8
                            radius: 4
                            color: alertScrollBar.pressed ? root.accentColor : root.mutedColor
                            opacity: alertScrollBar.pressed ? 1.0 : 0.75
                        }
                    }

                    delegate: Rectangle {
                        id: alertCard
                        // Luon chua san cho scrollbar. Truoc day width phu thuoc
                        // contentHeight; width lai doi wrap text -> doi height
                        // delegate -> doi contentHeight, tao binding loop va lam
                        // popup canh bao giat/khung khi danh sach vua tran.
                        width: Math.max(0, alertList.width - alertScrollBar.width - 4)
                        height: alertColumn.implicitHeight + 28
                        radius: 8
                        color: modelData.level === "ERROR" ? "#171824" : "#171b25"
                        border.color: root.borderColor
                        border.width: 1

                        Rectangle {
                            anchors {
                                top: parent.top
                                bottom: parent.bottom
                                left: parent.left
                                topMargin: 8
                                bottomMargin: 8
                            }
                            width: 4
                            radius: 2
                            color: modelData.level === "ERROR" ? root.errorColor
                                                                : root.warningColor
                        }

                        ColumnLayout {
                            id: alertColumn
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.leftMargin: 18
                            anchors.rightMargin: 14
                            anchors.topMargin: 14
                            spacing: 8

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 9

                                Rectangle {
                                    Layout.preferredWidth: severityText.implicitWidth + 16
                                    Layout.preferredHeight: 25
                                    radius: 12
                                    color: modelData.level === "ERROR" ? root.errorColor
                                                                        : root.warningColor
                                    Text {
                                        id: severityText
                                        anchors.centerIn: parent
                                        text: root.displayLevel(modelData.level)
                                        color: modelData.level === "ERROR" ? "#ffffff" : "#151006"
                                        font.pixelSize: 11
                                        font.bold: true
                                    }
                                }

                                Text {
                                    text: root.displayArea(modelData.area)
                                    color: root.accentColor
                                    font.pixelSize: 12
                                    font.bold: true
                                    font.letterSpacing: 0.8
                                }

                                Text {
                                    Layout.fillWidth: true
                                    text: modelData.title
                                    color: root.textColor
                                    font.pixelSize: 16
                                    font.bold: true
                                    elide: Text.ElideRight
                                }

                                Text {
                                    text: modelData.time
                                    color: root.mutedColor
                                    font.pixelSize: 12
                                }

                                Rectangle {
                                    visible: modelData.level === "WARNING" && modelData.acknowledged
                                    Layout.preferredWidth: acknowledgedText.implicitWidth + 16
                                    Layout.preferredHeight: 25
                                    radius: 12
                                    color: "#12352e"
                                    border.color: root.successColor
                                    Text {
                                        id: acknowledgedText
                                        anchors.centerIn: parent
                                        text: qsTr("ACKNOWLEDGED")
                                        color: root.successColor
                                        font.pixelSize: 10
                                        font.bold: true
                                    }
                                }
                            }

                            Text {
                                Layout.fillWidth: true
                                text: modelData.message
                                color: "#ffffff"
                                font.pixelSize: 14
                                wrapMode: Text.WordWrap
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: actionText.implicitHeight + 18
                                radius: 6
                                color: "#0b1929"
                                border.color: root.borderColor

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.margins: 9
                                    spacing: 9
                                    Text {
                                        text: qsTr("FIX")
                                        color: root.accentColor
                                        font.pixelSize: 11
                                        font.bold: true
                                    }
                                    Text {
                                        id: actionText
                                        Layout.fillWidth: true
                                        text: modelData.action
                                        color: root.mutedColor
                                        font.pixelSize: 13
                                        wrapMode: Text.WordWrap
                                    }
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true

                                Text {
                                    Layout.fillWidth: true
                                    text: modelData.level === "ERROR"
                                          ? qsTr("The error clears automatically when its source topic recovers.")
                                          : (modelData.acknowledged
                                             ? qsTr("Acknowledged; the warning remains visible until recovery.")
                                             : qsTr("Acknowledge the warning to confirm it was read and allow START."))
                                    color: modelData.level === "ERROR" ? root.errorColor
                                                                       : root.mutedColor
                                    font.pixelSize: 12
                                    font.bold: modelData.level === "ERROR"
                                    wrapMode: Text.WordWrap
                                }

                                MotionButton {
                                    visible: modelData.level === "WARNING" && !modelData.acknowledged
                                    Layout.preferredWidth: 132
                                    Layout.preferredHeight: 38
                                    text: qsTr("ACKNOWLEDGE")
                                    font.pixelSize: 11
                                    font.bold: true
                                    onClicked: systemAlertController.acknowledgeWarning(modelData.id)
                                    background: Rectangle {
                                        radius: 7
                                        color: parent.pressed ? Qt.darker(root.warningColor, 1.2)
                                                              : "#5a370d"
                                        border.color: root.warningColor
                                    }
                                    contentItem: Text {
                                        text: parent.text
                                        font: parent.font
                                        color: "#ffffff"
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
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
