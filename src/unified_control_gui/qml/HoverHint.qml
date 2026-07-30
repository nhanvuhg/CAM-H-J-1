import QtQuick 2.15

Rectangle {
    id: hint

    property string label: ""
    property color bg: "#e606101d"
    property color bc: "#163a52"
    property color tc: "#d6f1ff"

    width: hintText.implicitWidth + 16
    height: 22
    x: parent ? (parent.width - width) / 2 : 0
    y: parent ? parent.height + 6 : 0
    radius: 5
    color: bg
    border.color: bc
    border.width: 1
    z: 200

    Text {
        id: hintText
        anchors.centerIn: parent
        text: hint.label
        color: hint.tc
        font.pixelSize: 11
        font.bold: true
    }
}
