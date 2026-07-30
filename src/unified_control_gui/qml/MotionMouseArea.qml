import QtQuick 2.15
import QtGraphicalEffects 1.15

MouseArea {
    id: area

    property Item targetItem: parent
    property real hoverScale: 1.02
    property real pressScale: 0.97
    property int hoverDuration: 150
    property int pressDuration: 95
    property color shadowColor: "#66000000"
    property color pressedShadowColor: "#80000000"
    property color shimmerColor: "#55ffffff"
    property bool motionEnabled: true
    property bool shadowEnabled: false
    property bool shimmerEnabled: false
    property bool shimmerWhilePressed: false
    property bool raiseOnHover: false

    hoverEnabled: true
    cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor

    function applyMotion() {
        if (!targetItem) return
        var nextScale = (motionEnabled && enabled) ? (pressed ? pressScale : (containsMouse ? hoverScale : 1.0)) : 1.0
        if (motionEnabled) {
            targetItem.transformOrigin = Item.Center
            targetItem.z = raiseOnHover && (containsMouse || pressed || (shimmerEnabled && shimmerAnim.running)) ? 20 : 0
        }
        if (Math.abs(targetItem.scale - nextScale) > 0.001) {
            scaleAnim.stop()
            scaleAnim.from = targetItem.scale
            scaleAnim.to = nextScale
            scaleAnim.start()
        }
    }

    onContainsMouseChanged: applyMotion()
    onPressedChanged: {
        applyMotion()
        if (pressed && enabled && motionEnabled && shimmerEnabled) {
            shimmerAnim.restart()
        } else if (!pressed && shimmerWhilePressed) {
            shimmerAnim.stop()
        }
    }
    onEnabledChanged: applyMotion()
    Component.onCompleted: applyMotion()

    Connections {
        target: area.targetItem
        function onWidthChanged() { area.applyMotion() }
        function onHeightChanged() { area.applyMotion() }
    }

    Loader {
        id: shadowLoader
        anchors.fill: parent
        active: area.motionEnabled && area.shadowEnabled && area.enabled && (area.containsMouse || area.pressed || (area.shimmerEnabled && shimmerAnim.running))
        z: -1

        sourceComponent: DropShadow {
            anchors.fill: parent
            source: area.targetItem
            transparentBorder: true
            horizontalOffset: area.containsMouse ? 1 : 0
            verticalOffset: area.containsMouse ? 4 : 2
            radius: area.containsMouse ? 10 : 6
            samples: area.containsMouse ? 21 : 13
            color: area.pressed ? area.pressedShadowColor : area.shadowColor
        }
    }

    Item {
        anchors.fill: parent
        clip: true
        visible: area.motionEnabled && area.shimmerEnabled && shimmerAnim.running
        z: 100

        Rectangle {
            id: shimmer
            width: Math.max(parent.width * 0.34, 42)
            height: parent.height
            y: 0
            opacity: 0.32
            rotation: 0
            antialiasing: true
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.00; color: "transparent" }
                GradientStop { position: 0.42; color: "transparent" }
                GradientStop { position: 0.50; color: area.shimmerColor }
                GradientStop { position: 0.58; color: "transparent" }
                GradientStop { position: 1.00; color: "transparent" }
            }
        }
    }

    NumberAnimation {
        id: shimmerAnim
        target: shimmer
        property: "x"
        from: -area.width * 0.55
        to: area.width * 1.25
        duration: 780
        easing.type: Easing.InOutCubic
        onStopped: {
            if (area.shimmerWhilePressed && area.pressed && area.enabled && area.motionEnabled && area.shimmerEnabled) {
                shimmerAnim.restart()
            } else {
                area.applyMotion()
            }
        }
    }

    NumberAnimation {
        id: scaleAnim
        target: area.targetItem
        property: "scale"
        from: 1.0
        to: 1.0
        duration: area.pressed ? area.pressDuration : area.hoverDuration
        easing.type: area.pressed ? Easing.OutQuad : Easing.OutBack
    }

    onClicked: applyMotion()
}
