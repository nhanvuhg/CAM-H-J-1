import QtQuick 2.15
import QtQuick.Controls 2.15

// Nut dang nhap / dang xuat cho thanh cong cu.
//
// 14/08/2026 — vi sao can: AuthController::logout() da co san va dung, nhung
// KHONG cho nao trong QML goi no. Nut LOGIN duy nhat nam trong banner
// read-only cua Main.qml, ma banner do chi hien khi chua dang nhap hoac khi la
// viewer — admin/operator khong bao gio thay. Ket qua: dang nhap xong la het
// duong ra, popup dang nhap chi hien dung mot lan moi phien GUI, muon doi tai
// khoan phai khoi dong lai node GUI.
//
// Sau khi logout khong can tu mo popup: Main.qml da co Connections bat
// authenticatedChanged, thay authenticated == false thi goi openLoginDialog().
// Nen o day chi goi logout() roi de Main.qml lo phan con lai.
//
// Khong tham chieu mainWindow trong file nay: `mainWindow` la id khai bao
// trong Main.qml chu khong phai context property, id khong xuyen sang file
// component khac mot cach dam bao. Trang cha noi ho qua tin hieu loginRequested.
// authController thi an toan — no la context property dang ky trong main.cpp.
MotionButton {
    id: control

    // Phat khi bam luc DANG dang xuat. Trang cha noi vao mainWindow.openLoginDialog().
    signal loginRequested()

    property color borderColor: "#3ed0b4"
    property color gradientStart: "#1f9e86"
    property color gradientEnd: "#163a52"
    property color hintBorderColor: borderColor
    property color hintTextColor: "#d6f1ff"
    property int iconSize: 34

    readonly property bool signedIn: authController.authenticated

    hoverScale: 1.012
    pressScale: 0.99

    onClicked: {
        if (control.signedIn)
            authController.logout()
        else
            control.loginRequested()
    }

    background: Rectangle {
        radius: 6
        color: "transparent"
        border.color: control.borderColor
        border.width: 1
        gradient: Gradient {
            orientation: Gradient.Horizontal
            GradientStop {
                position: 0.0
                color: control.pressed ? Qt.darker(control.gradientStart, 1.15) : control.gradientStart
            }
            GradientStop {
                position: 1.0
                color: control.pressed ? Qt.darker(control.gradientEnd, 1.15) : control.gradientEnd
            }
        }
    }

    contentItem: Item {
        Image {
            anchors.centerIn: parent
            // Duong dan TUONG DOI co chu y: GUI nap QML thang tu cay nguon
            // (main.cpp:82), nen icon moi chay duoc ngay sau khi restart GUI.
            // Neu ghi "qrc:/qml/icons/user.svg" thi phai build lai moi co.
            // Van khai bao trong qml.qrc de nhanh du phong qrc cung chay.
            source: "icons/user.svg"
            width: control.iconSize
            height: control.iconSize
            fillMode: Image.PreserveAspectFit
            smooth: true
        }
        HoverHint {
            visible: control.hovered
            label: control.signedIn
                   ? qsTr("Log out") + (authController.username ? " (" + authController.username + ")" : "")
                   : qsTr("Log in")
            bc: control.hintBorderColor
            tc: control.hintTextColor
        }
    }
}
