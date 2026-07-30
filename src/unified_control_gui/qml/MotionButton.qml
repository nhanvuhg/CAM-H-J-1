import QtQuick 2.15
import QtQuick.Controls 2.15
import QtGraphicalEffects 1.15

Button {
    id: control

    property real hoverScale: 1.02
    property real pressScale: 0.97
    property int hoverDuration: 150
    property int pressDuration: 95
    property color shadowColor: "#66000000"
    property color pressedShadowColor: "#80000000"
    property color shimmerColor: "#55ffffff"
    property bool shadowEnabled: false
    property bool shimmerEnabled: false
    property bool raiseOnHover: false
    // "Độ lún" khi nhấn: dịch xuống + phủ tối nhẹ, áp dụng cho MỌI MotionButton
    // kể cả khi background tuỳ biến không tự xử lý trạng thái pressed.
    property real pressSinkOffset: 2
    property color pressTintColor: "#42000000"

    hoverEnabled: true
    transformOrigin: Item.Center
    scale: !enabled ? 1.0 : (pressed ? pressScale : (hovered ? hoverScale : 1.0))
    z: raiseOnHover && (hovered || pressed || (shimmerEnabled && shimmerAnim.running)) ? 20 : 0

    transform: Translate {
        y: control.enabled && control.pressed ? control.pressSinkOffset : 0
        Behavior on y { NumberAnimation { duration: control.pressDuration; easing.type: Easing.OutQuad } }
    }

    Rectangle {
        anchors.fill: parent
        z: 90
        radius: (control.background && control.background.radius !== undefined) ? control.background.radius : 6
        color: control.pressTintColor
        opacity: control.enabled && control.pressed ? 1.0 : 0.0
        Behavior on opacity { NumberAnimation { duration: 90 } }
    }

    layer.enabled: shadowEnabled && enabled && (hovered || pressed || (shimmerEnabled && shimmerAnim.running))
    layer.smooth: true
    layer.effect: DropShadow {
        transparentBorder: true
        horizontalOffset: control.hovered ? 1 : 0
        verticalOffset: control.hovered ? 4 : 2
        radius: control.hovered ? 10 : 6
        samples: control.hovered ? 21 : 13
        color: control.pressed ? control.pressedShadowColor : control.shadowColor
    }

    Behavior on scale {
        NumberAnimation {
            duration: control.pressed ? control.pressDuration : control.hoverDuration
            easing.type: control.pressed ? Easing.OutQuad : Easing.OutBack
        }
    }

    onPressedChanged: {
        if (pressed && enabled && shimmerEnabled) {
            shimmerAnim.restart()
        }
    }

    Item {
        anchors.fill: parent
        clip: true
        visible: control.shimmerEnabled && shimmerAnim.running
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
                GradientStop { position: 0.50; color: control.shimmerColor }
                GradientStop { position: 0.58; color: "transparent" }
                GradientStop { position: 1.00; color: "transparent" }
            }
        }
    }

    NumberAnimation {
        id: shimmerAnim
        target: shimmer
        property: "x"
        from: -control.width * 0.55
        to: control.width * 1.25
        duration: 780
        easing.type: Easing.InOutCubic
    }
}
