import QtQuick 2.15
import QtGraphicalEffects 1.15

// Lop "kinh mo" cho panel/popup: chup DUNG vung nam sau panel, lam mo, cat theo
// goc bo tron. Khong lam mo ca man hinh — chi vung panel che.
//
// Cach dung: dat lam con truc tiep cua panel Rectangle. Panel PHAI co alpha o
// mau nen, neu khong thi khong thay xuyen qua.
//
//     Rectangle {
//         radius: 22
//         gradient: ... // co alpha
//         FrostedBackdrop {
//             contentSource: stackView
//             captureRect: Qt.rect(popup.x, popup.y, popup.width, popup.height)
//             cornerRadius: 22
//         }
//     }
//
// Vi sao nam SAU phan to cua panel (z: -1) chu khong thay the panel: neu vi ly
// do nao lop blur khong ve duoc thi panel van con nguyen mau nen cua no, man
// dang nhap khong bao gio trang. Day la man hinh dang nhap cua may dang van
// hanh nen khong duoc phep co duong nao lam no bien mat.
//
// live: false — chi chup mot khung luc popup mo. Neu de live: true thi moi khung
// hinh Jetson phai chup lai va blur lai ca view camera dang chay.
Item {
    id: frost

    // Item chua thong tin phia sau (thuong la stackView).
    property Item contentSource: null
    // Vung can chup, theo toa do cua contentSource.
    property rect captureRect
    property int cornerRadius: 22
    // 64 la tran cua FastBlur trong Qt5.
    property int blurRadius: 64
    // Ha do phan giai texture: vua nhe hon vua khien cung ban kinh blur cho ra
    // do mo manh hon.
    property real downscale: 2

    anchors.fill: parent
    z: -1

    layer.enabled: true
    layer.effect: OpacityMask {
        maskSource: Item {
            width: frost.width
            height: frost.height
            Rectangle {
                anchors.fill: parent
                radius: frost.cornerRadius
            }
        }
    }

    // Blur mac vao layer cua chinh ShaderEffectSource, KHONG dat FastBlur thanh
    // item rieng lay snapshot lam source. Neu lam kieu do thi ca hai deu la item
    // duoc ve: ban net nam duoi, ban mo nam tren, va o vien — noi FastBlur nhat
    // dan vi lay mau ra ngoai bien — ban net se lo ra thanh mot vanh ro net.
    ShaderEffectSource {
        id: snapshot
        anchors.fill: parent
        sourceItem: frost.contentSource
        sourceRect: frost.captureRect
        live: false
        textureSize: Qt.size(Math.max(1, frost.width / frost.downscale),
                             Math.max(1, frost.height / frost.downscale))

        layer.enabled: true
        layer.effect: FastBlur { radius: frost.blurRadius }

        // Chup lai khi popup vua mo, va khi popup doi vi tri / doi chieu cao
        // (chieu cao thay doi khi dong bao loi xuat hien).
        Component.onCompleted: scheduleUpdate()
    }

    onCaptureRectChanged: snapshot.scheduleUpdate()
}
