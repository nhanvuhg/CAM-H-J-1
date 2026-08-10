import QtQuick 2.15
import QtQuick.Controls 2.15

// On-screen alphanumeric keyboard, the text counterpart to NumPad.
//
// Same commit rules and the same key styling: CANCEL restores the value the
// field held when the pad opened, OK applies what is on the pad, and a tap
// outside or Escape takes the CANCEL path.
//
// Opt-in per field rather than automatic. Fields fed by the barcode scanner
// (ink code, lot numbers) must not raise a keyboard on touch — the scanner
// types into whatever holds focus, and a popup would swallow the scan.
Popup {
    id: pad

    property Item target: null
    property string title: ""
    property string originalText: ""
    property string buffer: ""
    property int maxLength: 64
    // Sticky, not one-shot: pose names tend to be uniformly cased, so a
    // per-letter auto-reset would mean tapping shift for every character.
    property bool shifted: false

    signal applied(string value)
    signal cancelled()

    readonly property color cBg:      "#f20a1725"
    readonly property color cBorder:  "#263548"
    readonly property color cField:   "#123241"
    readonly property color cFieldBd: "#2b4a5c"
    readonly property color cKey:     "#07131f"
    readonly property color cKeyDown: "#67d0ff"
    readonly property color cKeyAlt:  "#0e2233"
    readonly property color cText:    "#ffffff"
    readonly property color cAccent:  "#67d0ff"
    readonly property color cDanger:  "#f0735c"
    readonly property color cOkStart: "#1a4a6e"
    readonly property color cOkEnd:   "#0c1726"

    property bool _committing: false

    modal: true
    dim: true
    focus: true
    closePolicy: Popup.CloseOnPressOutside | Popup.CloseOnEscape
    padding: 0

    width: 660
    height: body.implicitHeight + 30

    background: Rectangle {
        color: pad.cBg
        radius: 12
        border.color: pad.cBorder
        border.width: 1
    }

    readonly property real keyUnit: (width - 30 - 9 * 6) / 10

    component Key: Rectangle {
        id: keyCell
        property string label: ""
        property bool alt: false
        // Latched state, for SHIFT: reads as pressed while it is on.
        property bool active: false
        signal activated()

        readonly property bool lit: keyMA.pressed || active

        height: 50
        radius: 6
        color: lit ? pad.cKeyDown : (alt ? pad.cKeyAlt : pad.cKey)
        border.color: pad.cBorder
        border.width: 1

        Text {
            anchors.centerIn: parent
            text: keyCell.label
            color: keyCell.lit ? "#06101d" : pad.cText
            font.pixelSize: keyCell.label.length > 2 ? 14 : 19
            font.bold: true
        }
        MouseArea { id: keyMA; anchors.fill: parent; onClicked: keyCell.activated() }
    }

    function openFor(field, opts) {
        var o = opts || {}
        pad.target = field
        pad.title = o.title || ""
        pad.maxLength = o.maxLength || 64
        pad.originalText = (field && field.text !== undefined) ? String(field.text) : ""
        pad.buffer = pad.originalText
        pad.shifted = false
        pad._committing = false
        pad.open()
    }

    function type(ch) {
        if (buffer.length >= maxLength)
            return
        buffer += ch
    }

    function backspace() { buffer = buffer.slice(0, -1) }
    function clearAll()  { buffer = "" }

    function commit() {
        pad._committing = true
        if (pad.target && pad.target.text !== undefined)
            pad.target.text = pad.buffer
        pad.applied(pad.buffer)
        pad.close()
    }

    function revert() { pad.close() }

    onClosed: {
        if (!_committing) {
            if (pad.target && pad.target.text !== undefined)
                pad.target.text = pad.originalText
            pad.cancelled()
        }
        _committing = false
        pad.target = null
    }

    contentItem: FocusScope {
        focus: true

        Keys.onPressed: {
            if (event.key === Qt.Key_Return || event.key === Qt.Key_Enter) {
                pad.commit(); event.accepted = true
            } else if (event.key === Qt.Key_Backspace) {
                pad.backspace(); event.accepted = true
            } else if (event.text.length === 1 && event.text >= " ") {
                pad.type(event.text); event.accepted = true
            }
        }

        Column {
            id: body
            anchors { top: parent.top; left: parent.left; right: parent.right; margins: 15 }
            spacing: 8

            Text {
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                text: pad.title !== "" ? pad.title : qsTr("ENTER TEXT")
                color: pad.cAccent
                font.pixelSize: 16
                font.bold: true
                elide: Text.ElideRight
            }

            // ── Value display ──
            Rectangle {
                width: parent.width
                height: 50
                radius: 6
                color: pad.cField
                border.color: pad.cFieldBd
                border.width: 1

                Text {
                    anchors.left: parent.left
                    anchors.leftMargin: 12
                    anchors.right: countText.left
                    anchors.rightMargin: 8
                    anchors.verticalCenter: parent.verticalCenter
                    text: pad.buffer
                    color: pad.cAccent
                    font.pixelSize: 20
                    font.bold: true
                    elide: Text.ElideLeft
                }
                Text {
                    id: countText
                    anchors.right: parent.right
                    anchors.rightMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    text: pad.buffer.length + "/" + pad.maxLength
                    color: pad.cFieldBd
                    font.pixelSize: 12
                    font.bold: true
                }
            }

            // ── Digits ──
            Row {
                spacing: 6
                Repeater {
                    model: ["1","2","3","4","5","6","7","8","9","0"]
                    delegate: Key {
                        width: pad.keyUnit
                        label: modelData
                        onActivated: pad.type(modelData)
                    }
                }
            }

            // ── QWERTY ──
            Row {
                spacing: 6
                Repeater {
                    model: ["q","w","e","r","t","y","u","i","o","p"]
                    delegate: Key {
                        width: pad.keyUnit
                        label: pad.shifted ? modelData.toUpperCase() : modelData
                        onActivated: pad.type(label)
                    }
                }
            }

            Row {
                spacing: 6
                // Half-key indent, as on a physical keyboard.
                Item { width: pad.keyUnit / 2; height: 1 }
                Repeater {
                    model: ["a","s","d","f","g","h","j","k","l"]
                    delegate: Key {
                        width: pad.keyUnit
                        label: pad.shifted ? modelData.toUpperCase() : modelData
                        onActivated: pad.type(label)
                    }
                }
            }

            Row {
                spacing: 6
                Key {
                    width: pad.keyUnit * 1.5 + 3
                    label: qsTr("SHIFT")
                    alt: true
                    active: pad.shifted
                    onActivated: pad.shifted = !pad.shifted
                }
                Repeater {
                    model: ["z","x","c","v","b","n","m"]
                    delegate: Key {
                        width: pad.keyUnit
                        label: pad.shifted ? modelData.toUpperCase() : modelData
                        onActivated: pad.type(label)
                    }
                }
                Key {
                    width: pad.keyUnit * 1.5 + 3
                    label: "⌫"
                    alt: true
                    onActivated: pad.backspace()
                }
            }

            // ── Symbols and space ──
            Row {
                spacing: 6
                Key { width: pad.keyUnit; label: "-"; alt: true; onActivated: pad.type("-") }
                Key { width: pad.keyUnit; label: "_"; alt: true; onActivated: pad.type("_") }
                Key { width: pad.keyUnit; label: "."; alt: true; onActivated: pad.type(".") }
                Key {
                    width: pad.keyUnit * 5 + 4 * 6
                    label: qsTr("SPACE")
                    onActivated: pad.type(" ")
                }
                Key { width: pad.keyUnit * 2 + 6; label: qsTr("CLEAR"); alt: true; onActivated: pad.clearAll() }
            }

            // ── Action row ──
            Row {
                width: parent.width
                spacing: 10
                readonly property real halfW: (width - spacing) / 2

                Rectangle {
                    width: parent.halfW; height: 44; radius: 6
                    color: cancelMA.pressed
                           ? Qt.rgba(0.94, 0.27, 0.27, 0.32)
                           : Qt.rgba(0.94, 0.27, 0.27, 0.15)
                    border.color: pad.cDanger
                    border.width: 1
                    Text {
                        anchors.centerIn: parent; text: qsTr("CANCEL")
                        color: pad.cDanger; font.pixelSize: 14; font.bold: true
                    }
                    MouseArea { id: cancelMA; anchors.fill: parent; onClicked: pad.revert() }
                }
                Rectangle {
                    width: parent.halfW; height: 44; radius: 6
                    border.color: pad.cBorder
                    border.width: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: okMA.pressed ? Qt.darker(pad.cOkStart, 1.3) : pad.cOkStart }
                        GradientStop { position: 1.0; color: okMA.pressed ? Qt.darker(pad.cOkEnd, 1.3) : pad.cOkEnd }
                    }
                    Text {
                        anchors.centerIn: parent; text: qsTr("OK")
                        color: "#ffffff"; font.pixelSize: 14; font.bold: true
                    }
                    MouseArea { id: okMA; anchors.fill: parent; onClicked: pad.commit() }
                }
            }
        }
    }
}
