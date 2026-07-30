import QtQuick 2.15

TextInput {
    id: control

    property Item focusHost: null
    // Shared CartridgeSystem action-blue palette (same as SET buttons).
    property color focusBorderColor: "#1a4a6e"
    property int focusBorderRadius: 4
    property int focusBorderWidth: 2
    property bool showFocusBorder: true
    property bool focusBorderOnParent: true

    selectByMouse: false
    activeFocusOnTab: true
    selectionColor: focusBorderColor
    selectedTextColor: "#ffffff"
    inputMethodHints: validator ? Qt.ImhFormattedNumbersOnly : Qt.ImhNone

    onActiveFocusChanged: {
        if (activeFocus) {
            if (focusHost && focusHost.registerDataInput)
                focusHost.registerDataInput(control)
        } else {
            deselect()
            if (focusHost && focusHost.unregisterDataInput)
                focusHost.unregisterDataInput(control)
        }
    }

    Keys.onEscapePressed: {
        if (focusHost && focusHost.dismissDataInput)
            focusHost.dismissDataInput()
    }

    Component.onDestruction: {
        if (focusHost && focusHost.unregisterDataInput)
            focusHost.unregisterDataInput(control)
    }

    Rectangle {
        x: control.focusBorderOnParent && control.parent ? -control.x : 0
        y: control.focusBorderOnParent && control.parent ? -control.y : 0
        width: control.focusBorderOnParent && control.parent ? control.parent.width : control.width
        height: control.focusBorderOnParent && control.parent ? control.parent.height : control.height
        radius: control.focusBorderOnParent && control.parent && control.parent.radius !== undefined
                ? control.parent.radius : control.focusBorderRadius
        color: "transparent"
        border.color: control.focusBorderColor
        border.width: control.activeFocus ? control.focusBorderWidth : 0
        visible: control.showFocusBorder && control.activeFocus
        z: 1
    }

    MouseArea {
        anchors.fill: parent
        z: 2
        cursorShape: Qt.IBeamCursor
        onPressed: {
            if (!control.activeFocus) {
                control.forceActiveFocus(Qt.MouseFocusReason)
                control.selectAll()
            } else {
                control.deselect()
                control.cursorPosition = control.positionAt(mouse.x, mouse.y)
            }
            Qt.inputMethod.show()
            mouse.accepted = true
        }
    }
}
