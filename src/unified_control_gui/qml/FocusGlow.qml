import QtQuick 2.15
import QtGraphicalEffects 1.15

// Hao sang MEM quanh o nhap khi co focus.
//
// Vi sao khong dung mot Rectangle lon hon dat phia sau: no cho ra mot vanh teal
// SAC canh — dung nghia la mot cai vien, chinh thu ma kieu glass dang tranh.
// RectangularGlow la shader ve san do mo giam dan tu bien ra ngoai; no khong
// chup roi blur item nao nen re hon nhieu so voi Glow/DropShadow.
//
// Khong can chua cho cho phan loe: ShaderEffect ben trong RectangularGlow rong
// hon chinh item (parent.width + glowRadius*2 + cornerRadius*2) va duoc canh
// giua, nen glow tu tran ra ngoai bounds. Chi can anchors.fill o nhap.
//
// Cach dung: dat lam con cua background Rectangle cua TextField.
//
//     background: Rectangle {
//         radius: height / 2
//         gradient: ...
//         FocusGlow { active: myField.activeFocus }
//     }
RectangularGlow {
    id: glow

    property bool active: false

    anchors.fill: parent
    z: -1                       // nam sau phan to cua background

    color: "#3ed0b4"
    glowRadius: 12
    spread: 0.06

    // Bo goc cua vung phat sang phai cong them ban kinh loe, neu khong bon goc
    // se vuong ra ngoai duong bo tron cua o nhap. RectangularGlow tu ghim tran
    // o min(w, h) / 2 + glowRadius nen gia tri nay dung dung muc toi da.
    cornerRadius: height / 2 + glowRadius

    // 0.32: du thay o nhap nao dang go nhung khong choi mat. Day la num chinh
    // de dieu do sang — glowRadius/spread chi doi hinh dang vet loe.
    opacity: glow.active ? 0.32 : 0.0
    Behavior on opacity { NumberAnimation { duration: 150; easing.type: Easing.OutQuad } }
}
