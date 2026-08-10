import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    implicitWidth: 50
    implicitHeight: 50

    property color panelColor: "#0c1726"
    property color panelColorDeep: "#06101d"
    property color borderColor: "#163a52"
    property color textColor: "#d6f1ff"
    property color mutedColor: "#74899f"
    property color accentColor: "#7fcdf5"

    function openMenu() {
        var point = root.mapToItem(Overlay.overlay,
                                   root.width - languagePopup.width,
                                   root.height + 6)
        languagePopup.x = Math.max(8, Math.min(point.x,
            Overlay.overlay.width - languagePopup.width - 8))
        languagePopup.y = point.y
        languagePopup.open()
    }

    Button {
        id: languageButton
        anchors.fill: parent
        hoverEnabled: true
        Accessible.name: qsTr("Language")
        onClicked: root.openMenu()

        background: Rectangle {
            radius: 6
            color: languageButton.pressed
                   ? Qt.darker(root.panelColor, 1.16)
                   : (languageButton.hovered
                      ? Qt.lighter(root.panelColor, 1.10)
                      : root.panelColor)
            border.color: languagePopup.opened ? root.accentColor : root.borderColor
            border.width: 1
        }

        contentItem: Item {
            Image {
                anchors.centerIn: parent
                source: "qrc:/qml/icons/globe.svg"
                width: 28
                height: 28
                fillMode: Image.PreserveAspectFit
                smooth: true
            }

            Rectangle {
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                anchors.rightMargin: 2
                anchors.bottomMargin: 2
                width: 22
                height: 15
                radius: 7
                color: root.panelColorDeep
                border.color: root.borderColor
                border.width: 1
                Text {
                    anchors.centerIn: parent
                    text: languageController.language.toUpperCase()
                    color: root.accentColor
                    font.pixelSize: 9
                    font.bold: true
                }
            }

            HoverHint {
                visible: languageButton.hovered && !languagePopup.opened
                label: qsTr("Language")
                bc: root.borderColor
                tc: root.textColor
            }
        }
    }

    Popup {
        id: languagePopup
        parent: Overlay.overlay
        width: 190
        height: 132
        modal: false
        focus: true
        padding: 8
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        // Popup nổi trên nội dung trang nên nền phải đục hoàn toàn. Một số page
        // truyền vào màu card có alpha (CartridgePage: cCard #88…, cBg2 #99…),
        // làm chữ bên dưới lộ qua và rất khó đọc — bỏ alpha, giữ nguyên tông màu.
        readonly property color solidDeep: Qt.rgba(root.panelColorDeep.r,
                                                   root.panelColorDeep.g,
                                                   root.panelColorDeep.b, 1.0)
        readonly property color solidPanel: Qt.rgba(root.panelColor.r,
                                                    root.panelColor.g,
                                                    root.panelColor.b, 1.0)

        background: Rectangle {
            radius: 8
            color: languagePopup.solidDeep
            border.color: root.borderColor
            border.width: 1
        }

        contentItem: ColumnLayout {
            spacing: 6

            Text {
                Layout.fillWidth: true
                Layout.preferredHeight: 24
                text: qsTr("Language")
                color: root.textColor
                font.pixelSize: 13
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }

            Repeater {
                model: [
                    { code: "en", label: "English" },
                    { code: "vi", label: "Tiếng Việt" }
                ]

                delegate: Button {
                    id: optionButton
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    property bool selected: languageController.language === modelData.code
                    text: modelData.label
                    hoverEnabled: true
                    onClicked: {
                        languageController.setLanguage(modelData.code)
                        languagePopup.close()
                    }

                    background: Rectangle {
                        radius: 6
                        color: optionButton.selected
                               ? Qt.rgba(root.accentColor.r, root.accentColor.g,
                                         root.accentColor.b, 0.15)
                               : (optionButton.hovered ? languagePopup.solidPanel : "transparent")
                        border.color: optionButton.selected
                                      ? root.accentColor : root.borderColor
                        border.width: 1
                    }

                    contentItem: RowLayout {
                        spacing: 9
                        Rectangle {
                            Layout.preferredWidth: 8
                            Layout.preferredHeight: 8
                            radius: 4
                            color: optionButton.selected ? root.accentColor : root.mutedColor
                        }
                        Text {
                            Layout.fillWidth: true
                            text: optionButton.text
                            color: optionButton.selected ? root.textColor : root.mutedColor
                            font.pixelSize: 14
                            font.bold: optionButton.selected
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }
            }
        }
    }
}
