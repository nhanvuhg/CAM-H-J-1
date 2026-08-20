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
// Nut nay chi la CUA VAO, khong tu dang xuat. Bam vao se mo popup tai khoan
// (Main.qml) hien user dang thao tac, va nut LOG OUT nam trong popup do. Lam
// vay de mot cu bam nham tren man cam ung khong lam mat phien dang nhap.
//
// Khong tham chieu mainWindow trong file nay: `mainWindow` la id khai bao
// trong Main.qml chu khong phai context property, id khong xuyen sang file
// component khac mot cach dam bao. Trang cha noi ho qua tin hieu accountRequested.
// authController thi an toan — no la context property dang ky trong main.cpp.
MotionButton {
    id: control

    // Trang cha noi vao mainWindow.openAccountDialog(): dang nhap roi thi mo
    // popup tai khoan, chua dang nhap thi mo thang popup dang nhap.
    signal accountRequested()

    property color borderColor: "#3ed0b4"
    property color gradientStart: "#1f9e86"
    property color gradientEnd: "#163a52"
    property color hintBorderColor: borderColor
    property color hintTextColor: "#d6f1ff"
    property int iconSize: 34

    readonly property bool signedIn: authController.authenticated
    readonly property string operatorName:
        (authController.username || "").toString().toUpperCase()
    readonly property bool showName: signedIn && operatorName.length > 0

    // Do be rong ten o CAP GOC. Khong tham chieu id nam trong contentItem: id
    // ben trong do khong nhin thay tu day, bieu thuc se hong va tra ve 0.
    TextMetrics {
        id: nameMetrics
        font.pixelSize: 15
        font.bold: true
        text: control.operatorName
    }
    readonly property int desiredWidth:
        showName ? 14 + iconSize + 8 + Math.ceil(nameMetrics.width) + 14 : 50

    // Control co padding mac dinh, lam vung noi dung hep hon nut nen phan chu bi
    // day ra ngoai va nam duoi nut ben canh. Nen background phu tron ca icon lan
    // ten thi noi dung phai duoc dung tron be rong nut.
    padding: 0

    // Ten nguoi van hanh truoc day chi nam trong HoverHint — tren man cam ung
    // khong co "re chuot" nen thuc te khong ai doc duoc. Nut tu no rong ra vua
    // du chua ten; luc chua dang nhap giu nguyen o vuong 50px nhu cu.
    //

    hoverScale: 1.012
    pressScale: 0.99

    onClicked: control.accountRequested()

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
            label: control.signedIn && authController.username
                   ? qsTr("Account") + " (" + authController.username + ")"
                   : qsTr("Log in")
            bc: control.hintBorderColor
            tc: control.hintTextColor
        }
    }
}
