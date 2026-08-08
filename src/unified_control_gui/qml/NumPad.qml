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

    readonly property color cBg:      "#0c1726"
    readonly property color cPanel:   "#06101d"
    readonly property color cBorder:  "#1a4a6e"
    readonly property color cKey:     "#132a3f"
    readonly property color cKeyDown: "#1d3e5c"
    readonly property color cText:    "#e8f4ff"
    readonly property color cDim:     "#74899f"
    readonly property color cAccent:  "#36b6ff"
    readonly property color cOk:      "#1f9e86"
    readonly property color cCancel:  "#b53527"

    readonly property string displayText: edited ? (buffer === "" ? "0" : buffer) : originalText

    // Set only on the ENTER path so onClosed can tell a commit from an abandon.
    property bool _committing: false

    modal: true
    dim: true
    focus: true
    closePolicy: Popup.CloseOnPressOutside | Popup.CloseOnEscape
    padding: 0

    width: 340
    height: 452

    // One key cell; four of these across the pad body. A Grid cannot give the
    // zero key a double width or the action row its own height, so the keypad
    // is laid out as explicit rows of these instead.
    component Key: Rectangle {
        id: keyCell

        property string label: ""
        property string kind: "digit"     // digit | aux | cancel | enter
        property bool disabled: false
        signal activated()

        radius: 8
        opacity: disabled ? 0.32 : 1.0
        color: {
            if (kind === "enter")  return keyMA.pressed ? Qt.darker(pad.cOk, 1.25) : pad.cOk
            if (kind === "cancel") return keyMA.pressed ? Qt.darker(pad.cCancel, 1.25) : pad.cCancel
            return keyMA.pressed ? pad.cKeyDown : pad.cKey
        }
        border.color: pad.cBorder
        border.width: 1

        Text {
            anchors.centerIn: parent
            text: keyCell.label
            color: pad.cText
            font.pixelSize: (keyCell.kind === "cancel" || keyCell.kind === "enter") ? 17 : 23
            font.bold: true
            font.letterSpacing: (keyCell.kind === "cancel" || keyCell.kind === "enter") ? 1.2 : 0
        }

        MouseArea {
            id: keyMA
            anchors.fill: parent
            enabled: !keyCell.disabled
            onClicked: keyCell.activated()
        }
    }

    background: Rectangle {
        color: pad.cBg
        radius: 14
        border.color: pad.cBorder
        border.width: 2
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
            anchors.fill: parent
            anchors.margins: 14
            spacing: 10

            Text {
                width: parent.width
                text: pad.title
                visible: pad.title !== ""
                color: pad.cDim
                font.pixelSize: 14
                font.bold: true
                elide: Text.ElideRight
            }

            // ── Value display ──
            Rectangle {
                width: parent.width
                height: 58
                radius: 8
                color: pad.cPanel
                border.color: pad.edited ? pad.cAccent : pad.cBorder
                border.width: 2

                Text {
                    anchors.right: parent.right
                    anchors.rightMargin: 12
                    anchors.verticalCenter: parent.verticalCenter
                    text: pad.displayText + (pad.units ? " " + pad.units : "")
                    color: pad.cText
                    font.pixelSize: 28
                    font.bold: true
                    font.family: "monospace"
                }
            }

            // ── Digit block ──
            Column {
                id: keyBlock
                width: parent.width
                spacing: 8

                readonly property real cell: (width - spacing * 3) / 4
                readonly property real cellH: 54
                // Zero spans two columns, so it swallows the gap between them.
                readonly property real wideCell: cell * 2 + spacing

                Row {
                    spacing: keyBlock.spacing
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "7"; onActivated: pad.press("7") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "8"; onActivated: pad.press("8") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "9"; onActivated: pad.press("9") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "⌫"; kind: "aux"
                          onActivated: pad.backspace() }
                }
                Row {
                    spacing: keyBlock.spacing
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "4"; onActivated: pad.press("4") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "5"; onActivated: pad.press("5") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "6"; onActivated: pad.press("6") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "C"; kind: "aux"
                          onActivated: pad.clearAll() }
                }
                Row {
                    spacing: keyBlock.spacing
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "1"; onActivated: pad.press("1") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "2"; onActivated: pad.press("2") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "3"; onActivated: pad.press("3") }
                    Key { width: keyBlock.cell; height: keyBlock.cellH; label: "±"; kind: "aux"
                          disabled: !pad.allowSign; onActivated: pad.press("-") }
                }
                Row {
                    spacing: keyBlock.spacing
                    Key { width: keyBlock.wideCell; height: keyBlock.cellH; label: "0"
                          onActivated: pad.press("0") }
                    Key { width: keyBlock.wideCell; height: keyBlock.cellH; label: "."
                          disabled: !pad.allowDecimal; onActivated: pad.press(".") }
                }
            }

            // ── Action row ──
            // Held apart from the digits so a mis-tap while typing cannot
            // commit or discard the value.
            Item { width: 1; height: 4 }

            Row {
                width: parent.width
                spacing: 10

                readonly property real halfW: (width - spacing) / 2

                Key {
                    width: parent.halfW; height: 58
                    label: "CANCEL"; kind: "cancel"
                    onActivated: pad.revert()
                }
                Key {
                    width: parent.halfW; height: 58
                    label: "ENTER"; kind: "enter"
                    onActivated: pad.commit()
                }
            }
        }
    }
}
