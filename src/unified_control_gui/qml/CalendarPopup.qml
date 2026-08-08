import QtQuick 2.15
import QtQuick.Controls 2.15

// Month picker for the production date fields.
//
// Qt.labs.calendar is not deployed on this Jetson (there is no Qt/labs module
// directory at all under the Qt 5.15 qml path), so MonthGrid is unavailable and
// the month grid is built here from plain date arithmetic.
//
// Dismissal rule, as specified by the operator: tapping outside closes the
// calendar and applies the day most recently tapped. The day tap therefore only
// records a pending selection; `accepted` fires once, on close. That keeps a
// tap-and-look-around gesture from firing five HTTP reloads.
Popup {
    id: cal

    // ISO yyyy-MM-dd the field currently holds; seeds the visible month.
    property string selectedIso: ""
    // Day tapped since the popup opened, applied on close. Empty means the
    // operator only browsed months, in which case nothing is applied.
    property string pendingIso: ""

    property int viewYear: 0
    property int viewMonth: 0   // 0-based, matches JS Date

    signal accepted(string isoDate)

    readonly property color cBg:      "#0c1726"
    readonly property color cPanel:   "#06101d"
    readonly property color cBorder:  "#1a4a6e"
    readonly property color cText:    "#e8f4ff"
    readonly property color cDim:     "#74899f"
    readonly property color cAccent:  "#36b6ff"
    readonly property color cToday:   "#f5a623"
    readonly property color cCell:    "#132a3f"
    readonly property color cCellDown:"#1d3e5c"

    readonly property var monthNames: ["January", "February", "March", "April",
                                       "May", "June", "July", "August",
                                       "September", "October", "November", "December"]

    modal: false
    focus: true
    closePolicy: Popup.CloseOnPressOutside | Popup.CloseOnEscape
    padding: 0
    width: 322
    height: 358

    background: Rectangle {
        color: cal.cBg
        radius: 12
        border.color: cal.cBorder
        border.width: 2
    }

    function isoOf(year, month, day) {
        var m = month + 1
        return year + "-" + (m < 10 ? "0" + m : m) + "-" + (day < 10 ? "0" + day : day)
    }

    function openAt(iso) {
        var seed = new Date()
        var parts = String(iso || "").split("-")
        if (parts.length === 3) {
            var y = parseInt(parts[0], 10)
            var mo = parseInt(parts[1], 10)
            var d = parseInt(parts[2], 10)
            if (!isNaN(y) && !isNaN(mo) && !isNaN(d))
                seed = new Date(y, mo - 1, d)
        }
        cal.selectedIso = String(iso || "")
        cal.pendingIso = ""
        cal.viewYear = seed.getFullYear()
        cal.viewMonth = seed.getMonth()
        cal.open()
    }

    function shiftMonth(delta) {
        var d = new Date(viewYear, viewMonth + delta, 1)
        viewYear = d.getFullYear()
        viewMonth = d.getMonth()
    }

    // Monday-first grid: JS getDay() is Sunday-based, so rotate it.
    readonly property int leadingBlanks: {
        var jsDay = new Date(viewYear, viewMonth, 1).getDay()
        return (jsDay + 6) % 7
    }
    readonly property int daysInMonth: new Date(viewYear, viewMonth + 1, 0).getDate()
    readonly property string todayIso: Qt.formatDate(new Date(), "yyyy-MM-dd")

    onClosed: {
        if (cal.pendingIso !== "") {
            var applied = cal.pendingIso
            cal.pendingIso = ""
            cal.accepted(applied)
        }
    }

    contentItem: Item {

        Column {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 8

            // ── Month header ──
            Item {
                width: parent.width
                height: 38

                Rectangle {
                    id: prevBtn
                    width: 38; height: 38; radius: 8
                    anchors.left: parent.left
                    color: prevMA.pressed ? cal.cCellDown : cal.cCell
                    border.color: cal.cBorder; border.width: 1
                    Text {
                        anchors.centerIn: parent; text: "‹"
                        color: cal.cText; font.pixelSize: 22; font.bold: true
                    }
                    MouseArea {
                        id: prevMA; anchors.fill: parent
                        onClicked: cal.shiftMonth(-1)
                    }
                }

                Text {
                    anchors.centerIn: parent
                    text: cal.monthNames[cal.viewMonth] + " " + cal.viewYear
                    color: cal.cText; font.pixelSize: 16; font.bold: true
                }

                Rectangle {
                    id: nextBtn
                    width: 38; height: 38; radius: 8
                    anchors.right: parent.right
                    color: nextMA.pressed ? cal.cCellDown : cal.cCell
                    border.color: cal.cBorder; border.width: 1
                    Text {
                        anchors.centerIn: parent; text: "›"
                        color: cal.cText; font.pixelSize: 22; font.bold: true
                    }
                    MouseArea {
                        id: nextMA; anchors.fill: parent
                        onClicked: cal.shiftMonth(1)
                    }
                }
            }

            // ── Weekday header ──
            Row {
                width: parent.width
                spacing: 4
                Repeater {
                    model: ["Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"]
                    delegate: Text {
                        width: (parent.width - 6 * 4) / 7
                        horizontalAlignment: Text.AlignHCenter
                        text: modelData
                        color: cal.cDim
                        font.pixelSize: 12
                        font.bold: true
                    }
                }
            }

            // ── Day grid ──
            Grid {
                id: dayGrid
                width: parent.width
                columns: 7
                spacing: 4

                readonly property real cellW: (width - spacing * 6) / 7
                readonly property real cellH: 36

                Repeater {
                    // Six rows always, so the popup does not resize between
                    // months and shift the cell the finger is aiming at.
                    model: 42

                    delegate: Item {
                        id: dayCell

                        readonly property int dayNumber: index - cal.leadingBlanks + 1
                        readonly property bool inMonth: dayNumber >= 1 && dayNumber <= cal.daysInMonth
                        readonly property string iso: inMonth
                                                      ? cal.isoOf(cal.viewYear, cal.viewMonth, dayNumber)
                                                      : ""
                        readonly property bool isSelected: inMonth
                                                           && iso === (cal.pendingIso !== ""
                                                                       ? cal.pendingIso : cal.selectedIso)
                        readonly property bool isToday: inMonth && iso === cal.todayIso

                        width: dayGrid.cellW
                        height: dayGrid.cellH

                        Rectangle {
                            anchors.fill: parent
                            radius: 7
                            visible: dayCell.inMonth
                            color: dayCell.isSelected
                                   ? cal.cAccent
                                   : (dayMA.pressed ? cal.cCellDown : cal.cCell)
                            border.color: dayCell.isToday ? cal.cToday : cal.cBorder
                            border.width: dayCell.isToday ? 2 : 1

                            Text {
                                anchors.centerIn: parent
                                text: dayCell.inMonth ? dayCell.dayNumber : ""
                                color: dayCell.isSelected ? "#06101d" : cal.cText
                                font.pixelSize: 14
                                font.bold: dayCell.isSelected || dayCell.isToday
                            }

                            MouseArea {
                                id: dayMA
                                anchors.fill: parent
                                enabled: dayCell.inMonth
                                // Records the choice only; onClosed applies it.
                                onClicked: cal.pendingIso = dayCell.iso
                            }
                        }
                    }
                }
            }

            // ── Today shortcut ──
            Rectangle {
                width: parent.width
                height: 36
                radius: 8
                color: todayMA.pressed ? cal.cCellDown : cal.cCell
                border.color: cal.cBorder; border.width: 1
                Text {
                    anchors.centerIn: parent
                    text: "TODAY"
                    color: cal.cText; font.pixelSize: 13; font.bold: true
                }
                MouseArea {
                    id: todayMA
                    anchors.fill: parent
                    onClicked: {
                        var now = new Date()
                        cal.viewYear = now.getFullYear()
                        cal.viewMonth = now.getMonth()
                        cal.pendingIso = cal.todayIso
                    }
                }
            }
        }
    }
}
