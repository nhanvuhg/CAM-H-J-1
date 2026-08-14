import QtQuick 2.15

// Specular edge highlight — dải "glass" sheen 1px ở mép panel, đồng bộ với
// CameraPage. Thuần trang trí, không có logic.
// Cách dùng: đặt `GlassHighlight {}` làm con trực tiếp của panel Rectangle.
//
// 14/08/2026 thêm `atBottom` và `inset`. Mặc định giữ y nguyên hành vi cũ
// (mép trên, thụt 2px) nên 14 chỗ đang dùng không đổi một pixel nào.
//
//   inset  — cần cho panel bo góc NHIỀU. Trên khung dạng viên thuốc
//            (radius = height/2) mép trên chỉ phẳng ở đoạn giữa, dải sheen kéo
//            hết bề ngang sẽ chọi ra ngoài đường cong và trông rời khỏi khung.
//            Truyền `inset: parent.height / 2` thì dải đúng bằng đoạn phẳng và
//            tự co theo chiều cao panel.
//
//   atBottom — mép dưới thay vì mép trên. Dùng cho phần tử trông LÕM xuống
//            (ô nhập): vật lõm thì mép trên bị đổ bóng còn ánh sáng đọng ở mép
//            dưới, nên sheen ở dưới mới đúng. Phần tử nổi (nút) giữ mép trên.
Rectangle {
    id: sheen

    property bool atBottom: false
    property int inset: 2

    anchors.left: parent.left
    anchors.right: parent.right
    anchors.leftMargin: sheen.inset
    anchors.rightMargin: sheen.inset

    anchors.top: sheen.atBottom ? undefined : parent.top
    anchors.bottom: sheen.atBottom ? parent.bottom : undefined
    anchors.topMargin: 1
    anchors.bottomMargin: 1

    height: 1
    gradient: Gradient {
        orientation: Gradient.Horizontal
        GradientStop { position: 0.0; color: "transparent" }
        GradientStop { position: 0.35; color: "#55ffffff" }
        GradientStop { position: 0.65; color: "#55ffffff" }
        GradientStop { position: 1.0; color: "transparent" }
    }
}
