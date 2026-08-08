import QtQuick 2.15
import QtQuick.Controls 2.15

// Touch numeric keypad for the panel display.
//
// The Jetson runs without an on-screen keyboard, so a numeric field could only
// be edited from a physical keyboard. This popup is opened by SmartTextField /
// SmartTextInput whenever a numeric field is touched.
//
// Commit rules, as specified by the operator:
//   CANCEL  restores the value the field held when the pad opened
//   ENTER   applies what is on the pad
// CANCEL therefore always leaves the field exactly as it was found, whether or
// not any digit was typed. Tapping outside and Escape take the same path — an
// accidental tap must never rewrite a machine parameter. Every exit that is not
// ENTER funnels through onClosed, so there is exactly one revert path.
Popup {
    id: pad

    // Field currently being edited; set through openFor(), never bound.
    property Item target: null
    property string title: ""
    property string units: ""
    // Value at open time — what CANCEL restores.
    property string originalText: ""
    property string buffer: ""
    // True once the operator touches a key, so the display shows the live
    // buffer instead of the inherited value.
    property bool edited: false
    property bool allowDecimal: true
    property bool allowSign: true

    signal applied(string value)
    signal cancelled()

    // Matches the ENTER VALUE keypad in InkTab so the panel has one keypad
    // look, with one deliberate change: the key faces are darker than that
    // one's, which sat close enough to the page background to read as flat.
    readonly property color cBg:      "#f20a1725"   // popup body
    readonly property color cBorder:  "#263548"     // cFrameBorder
    readonly property color cField:   "#123241"     // value display face
    readonly property color cFieldBd: "#2b4a5c"     // cFieldBorder
    readonly property color cKey:     "#07131f"     // key face — darkest layer
    readonly property color cKeyDown: "#67d0ff"     // pressed flashes accent
    readonly property color cText:    "#ffffff"
    readonly property color cAccent:  "#67d0ff"
    readonly property color cDanger:  "#f0735c"
    readonly property color cOkStart: "#1a4a6e"
    readonly property color cOkEnd:   "#0c1726"

    readonly property string displayText: edited ? (buffer === "" ? "0" : buffer) : originalText

    // Set only on the ENTER path so onClosed can tell a commit from an abandon.
    property bool _committing: false

    modal: true
    dim: true
    focus: true
    closePolicy: Popup.CloseOnPressOutside | Popup.CloseOnEscape
    padding: 0

    width: 320
    // Derived rather than a magic number, so a key-size change cannot push the
    // action row out through the bottom of the popup.
    height: body.implicitHeight + 30

    background: Rectangle {
        color: pad.cBg
        radius: 12
        border.color: pad.cBorder
        border.width: 1
    }

    // One key face. Pressing flashes the accent colour, matching InkTab.
    component Key: Rectangle {
        id: keyCell

        property string label: ""
        property bool disabled: false
        signal activated()

        radius: 6
        opacity: disabled ? 0.3 : 1.0
        color: keyMA.pressed ? pad.cKeyDown : pad.cKey
        border.color: pad.cBorder
        border.width: 1

        Text {
            anchors.centerIn: parent
            text: keyCell.label
            color: keyMA.pressed ? "#06101d" : pad.cText
            font.pixelSize: 20
            font.bold: true
        }

        MouseArea {
            id: keyMA
            anchors.fill: parent
            enabled: !keyCell.disabled
            onClicked: keyCell.activated()
        }
    }

    function openFor(field, opts) {
        var o = opts || {}
        pad.target = field
        pad.title = o.title || ""
        pad.units = o.units || ""
        pad.allowDecimal = o.allowDecimal === undefined ? true : o.allowDecimal
        pad.allowSign = o.allowSign === undefined ? true : o.allowSign
        pad.originalText = (field && field.text !== undefined) ? String(field.text) : ""
        pad.buffer = pad.originalText
        pad.edited = false
        pad._committing = false
        pad.open()
    }

    function press(key) {
        if (!edited) {
            // The first keypress replaces the inherited value rather than
            // appending to it — typing "5" into a field showing 120.5 must
            // mean 5, not 120.55.
            buffer = ""
            edited = true
        }
        if (key === ".") {
            if (!allowDecimal || buffer.indexOf(".") >= 0)
                return
            buffer = (buffer === "" || buffer === "-") ? buffer + "0." : buffer + "."
            return
        }
        if (key === "-") {
            if (!allowSign)
                return
            buffer = buffer.charAt(0) === "-" ? buffer.slice(1) : "-" + buffer
            return
        }
        buffer += key
    }

    function backspace() {
        if (!edited) {
            buffer = ""
            edited = true
            return
        }
        buffer = buffer.slice(0, -1)
    }

    function clearAll() {
        buffer = ""
        edited = true
    }

    function commit() {
        var value = edited ? buffer : originalText
        // A buffer holding only a sign or a lone dot is not a number; fall back
        // rather than writing "-" into a parameter.
        if (value === "" || value === "-" || value === "." || value === "-.")
            value = originalText
        pad._committing = true
        if (pad.target && pad.target.text !== undefined)
            pad.target.text = value
        pad.applied(value)
        pad.close()
    }

    // CANCEL just closes; onClosed performs the single revert.
    function revert() {
        pad.close()
    }

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

        // Physical keyboard still works when one is attached for servicing.
        Keys.onPressed: {
            if (event.key === Qt.Key_Return || event.key === Qt.Key_Enter) {
                pad.commit(); event.accepted = true
            } else if (event.key === Qt.Key_Backspace) {
                pad.backspace(); event.accepted = true
            } else if (event.key === Qt.Key_Delete) {
                pad.clearAll(); event.accepted = true
            } else if (event.text.length === 1 && "0123456789.-".indexOf(event.text) >= 0) {
                pad.press(event.text); event.accepted = true
            }
        }

        Column {
            id: body
            // Three-sided anchor keeps implicitHeight driven by the children,
            // which is what sizes the popup above.
            anchors { top: parent.top; left: parent.left; right: parent.right; margins: 15 }
            spacing: 8

            // ── Title ──
            Text {
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                text: pad.title !== ""
                      ? pad.title + (pad.units ? " (" + pad.units + ")" : "")
                      : "ENTER VALUE" + (pad.units ? " (" + pad.units + ")" : "")
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
                    anchors.centerIn: parent
                    text: pad.displayText
                    color: pad.cAccent
                    font.pixelSize: 24
                    font.bold: true
                }

                // The reference keypad has no sign key and no room for one in a
                // 3x4 grid. Fields whose validator allows negatives still need
                // it, so it lives on the display instead of costing a key.
                Rectangle {
                    visible: pad.allowSign
                    anchors.left: parent.left
                    anchors.leftMargin: 6
                    anchors.verticalCenter: parent.verticalCenter
                    width: 38; height: 38; radius: 6
                    color: signMA.pressed ? pad.cKeyDown : pad.cKey
                    border.color: pad.cBorder
                    border.width: 1
                    Text {
                        anchors.centerIn: parent
                        text: "\u00b1"
                        color: signMA.pressed ? "#06101d" : pad.cText
                        font.pixelSize: 18
                        font.bold: true
                    }
                    MouseArea {
                        id: signMA
                        anchors.fill: parent
                        onClicked: pad.press("-")
                    }
                }
            }

            // ── Digit block: same 3x4 arrangement as the InkTab keypad ──
            Grid {
                id: keyGrid
                width: parent.width
                columns: 3
                spacing: 6

                readonly property real cell: (width - spacing * 2) / 3
                readonly property real cellH: 54

                Repeater {
                    model: ["7", "8", "9",
                            "4", "5", "6",
                            "1", "2", "3",
                            ".", "0", "\u232b"]

                    delegate: Key {
                        width: keyGrid.cell
                        height: keyGrid.cellH
                        label: modelData
                        disabled: modelData === "." && !pad.allowDecimal
                        onActivated: {
                            if (modelData === "\u232b")      pad.backspace()
                            else                              pad.press(modelData)
                        }
                    }
                }
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
                        anchors.centerIn: parent
                        text: "CANCEL"
                        color: pad.cDanger
                        font.pixelSize: 14
                        font.bold: true
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
                        anchors.centerIn: parent
                        text: "OK"
                        color: "#ffffff"
                        font.pixelSize: 14
                        font.bold: true
                    }
                    MouseArea { id: okMA; anchors.fill: parent; onClicked: pad.commit() }
                }
            }
        }
    }
}
