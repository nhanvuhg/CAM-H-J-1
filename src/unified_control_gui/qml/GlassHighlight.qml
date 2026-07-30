import QtQuick 2.15

// Specular top-edge highlight — dải "glass" sheen 1px ở mép trên panel,
// đồng bộ với CameraPage. Thuần trang trí, không có logic.
// Cách dùng: đặt `GlassHighlight {}` làm con trực tiếp của panel Rectangle.
Rectangle {
    anchors { top: parent.top; left: parent.left; right: parent.right }
    anchors.topMargin: 1
    anchors.leftMargin: 2
    anchors.rightMargin: 2
    height: 1
    gradient: Gradient {
        orientation: Gradient.Horizontal
        GradientStop { position: 0.0; color: "transparent" }
        GradientStop { position: 0.35; color: "#55ffffff" }
        GradientStop { position: 0.65; color: "#55ffffff" }
        GradientStop { position: 1.0; color: "transparent" }
    }
}
