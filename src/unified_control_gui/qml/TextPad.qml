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
        // insert()/remove() ghi thang vao TextInput nen pha binding text:buffer.
        // Vi vay moi lan mo phai nap lai bang tay, va dat con tro o cuoi de go
        // tiep duoc ngay nhu truoc.
        var input = _valueInput()
        if (input) {
            input.text = pad.buffer
            input.cursorPosition = input.text.length
            input.forceActiveFocus()
        }
    }

    // Ky tu roi vao vi tri con tro, khong phai cuoi chuoi. Tim input qua
    // objectName vi no nam trong contentItem, khoi tao sau cac ham nay.
    function _valueInput() {
        return pad.contentItem ? _findValue(pad.contentItem) : null
    }

    function _findValue(item) {
        if (!item) return null
        if (item.objectName === "textPadValue") return item
        for (var i = 0; i < item.children.length; ++i) {
            var found = _findValue(item.children[i])
            if (found) return found
        }
        return null
    }

    function type(ch) {
        if (buffer.length >= maxLength)
            return
        var input = _valueInput()
        if (!input) { buffer += ch; return }
        if (input.selectedText.length > 0)
            input.remove(input.selectionStart, input.selectionEnd)
        input.insert(input.cursorPosition, ch)
    }

    function backspace() {
        var input = _valueInput()
        if (!input) { buffer = buffer.slice(0, -1); return }
        if (input.selectedText.length > 0) { input.remove(input.selectionStart, input.selectionEnd); return }
        if (input.cursorPosition <= 0) return
        input.remove(input.cursorPosition - 1, input.cursorPosition)
    }

    function clearAll() {
        buffer = ""
        var input = _valueInput()
        if (input) { input.text = ""; input.cursorPosition = 0 }
    }

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

                // TextInput chu khong phai Text: can con tro de chen/xoa giua
                // chuoi. Ban cu chi la nhan tinh, moi phim deu tac dong vao
                // cuoi buffer, nen go sai mot ky tu o dau ten la phai xoa het
                // phan sau roi go lai.
                TextInput {
                    id: valueInput
                    objectName: "textPadValue"
                    anchors.left: parent.left
                    anchors.leftMargin: 12
                    anchors.right: countText.left
                    anchors.rightMargin: 8
                    anchors.verticalCenter: parent.verticalCenter
                    color: pad.cAccent
                    font.pixelSize: 20
                    font.bold: true
                    clip: true
                    maximumLength: pad.maxLength
                    // MouseArea ben duoi lo cham; TextInput khong tu xu ly nua.
                    activeFocusOnPress: false
                    selectByMouse: false
                    cursorVisible: true
                    onTextChanged: if (pad.buffer !== text) pad.buffer = text

                    // Vach nhay day hon mac dinh 1 px — tren panel cam ung nhin
                    // xa, vach mot pixel gan nhu khong thay.
                    cursorDelegate: Rectangle {
                        width: 3
                        radius: 1
                        color: pad.cAccent
                        visible: valueInput.cursorVisible
                        SequentialAnimation on opacity {
                            loops: Animation.Infinite
                            running: valueInput.cursorVisible
                            NumberAnimation { to: 0; duration: 500 }
                            NumberAnimation { to: 1; duration: 500 }
                        }
                    }

                    // Cham de dat con tro, giu va keo de ro dan — giong dien
                    // thoai. Vung cham cao hon chu de ngon tay khong truot ra.
                    MouseArea {
                        anchors.fill: parent
                        anchors.topMargin: -12
                        anchors.bottomMargin: -12
                        preventStealing: true
                        onPressed: {
                            valueInput.forceActiveFocus()
                            valueInput.cursorPosition =
                                valueInput.positionAt(mouse.x, valueInput.height / 2, TextInput.CursorBetweenCharacters)
                        }
                        onPositionChanged: {
                            if (!pressed) return
                            valueInput.cursorPosition =
                                valueInput.positionAt(mouse.x, valueInput.height / 2, TextInput.CursorBetweenCharacters)
                        }
                    }
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
