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
                sourceSize.width: 72
                sourceSize.height: 72
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
                       ? "Cảnh báo hệ thống"
                       : (root.hasError
                          ? systemAlertController.errorCount + " lỗi đang chặn START"
                          : (root.needsAcknowledge
                             ? systemAlertController.unacknowledgedWarningCount
                               + " cảnh báo cần acknowledge"
                             : systemAlertController.warningCount
                               + " cảnh báo đã acknowledge"))
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
                        sourceSize.width: 76
                        sourceSize.height: 76
                        fillMode: Image.PreserveAspectFit
                    }

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 3

                        Text {
                            text: "SYSTEM WARNINGS & ERRORS"
                            color: root.textColor
                            font.pixelSize: 20
                            font.bold: true
                            font.letterSpacing: 1.1
                        }
                        Text {
                            text: root.totalCount === 0
                                  ? "Không có cảnh báo đang hoạt động"
                                  : systemAlertController.errorCount + " ERROR  -  "
                                    + systemAlertController.warningCount + " WARNING"
                            color: root.mutedColor
                            font.pixelSize: 13
                        }
                    }

                    MotionButton {
                        visible: systemAlertController.unacknowledgedWarningCount > 1
                        Layout.preferredWidth: 168
                        Layout.preferredHeight: 42
                        text: "ACK ALL WARNINGS"
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
                        text: "HỆ THỐNG KHÔNG CÓ CẢNH BÁO ACTIVE"
                        color: root.textColor
                        font.pixelSize: 16
                        font.bold: true
                    }
                    Text {
                        Layout.alignment: Qt.AlignHCenter
                        text: "Lịch sử lỗi trong AUTO/AI vẫn được lưu tại Production Output."
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

                    ScrollBar.vertical: ScrollBar {
                        policy: ScrollBar.AsNeeded
                    }

                    delegate: Rectangle {
                        id: alertCard
                        width: alertList.width - (alertList.contentHeight > alertList.height ? 12 : 0)
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
                                        text: modelData.level
                                        color: modelData.level === "ERROR" ? "#ffffff" : "#151006"
                                        font.pixelSize: 11
                                        font.bold: true
                                    }
                                }

                                Text {
                                    text: modelData.area
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
                                        text: "ACKNOWLEDGED"
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
                                        text: "FIX"
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
                                          ? "ERROR tự mở khóa khi topic nguồn báo đã hồi phục."
                                          : (modelData.acknowledged
                                             ? "Đã acknowledge; cảnh báo vẫn hiển thị tới khi hồi phục."
                                             : "Acknowledge xác nhận đã đọc cảnh báo và cho phép START.")
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
                                    text: "ACKNOWLEDGE"
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
