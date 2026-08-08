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

    // Swaps the day grid for the month/year wheels.
    property bool pickerMode: false

    signal accepted(string isoDate)

    readonly property color cBg:       "#f20a1725"
    readonly property color cBorder:   "#263548"
    readonly property color cText:     "#e8f4ff"
    readonly property color cDim:      "#74899f"
    readonly property color cAccent:   "#67d0ff"
    readonly property color cToday:    "#f5a623"
    readonly property color cCell:     "#07131f"
    readonly property color cCellDown: "#123241"

    readonly property var monthNames: ["January", "February", "March", "April",
                                       "May", "June", "July", "August",
                                       "September", "October", "November", "December"]

    // Range offered by the year wheel. Production logs do not go back further
    // than the machine's install year, and a couple of years ahead covers a
    // clock that has drifted forward.
    readonly property var yearList: {
        var list = []
        var now = new Date().getFullYear()
        for (var y = now - 6; y <= now + 2; y++)
            list.push(y)
        return list
    }

    modal: false
    focus: true
    closePolicy: Popup.CloseOnPressOutside | Popup.CloseOnEscape
    padding: 0
    width: 322
    // Derived, not a magic number: the fixed height used before cut the TODAY
    // button off at the bottom, and the two modes are different heights.
    height: body.implicitHeight + 24

    background: Rectangle {
        color: cal.cBg
        radius: 12
        border.color: cal.cBorder
        border.width: 1
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
        cal.pickerMode = false
        cal.open()
    }

    function shiftMonth(delta) {
        var d = new Date(viewYear, viewMonth + delta, 1)
        viewYear = d.getFullYear()
        viewMonth = d.getMonth()
    }

    function togglePicker() {
        if (!cal.pickerMode) {
            // Seed the wheels imperatively; binding currentIndex to viewMonth
            // while also writing it back would be a two-way binding.
            monthWheel.currentIndex = cal.viewMonth
            var yi = cal.yearList.indexOf(cal.viewYear)
            yearWheel.currentIndex = yi >= 0 ? yi : Math.floor(cal.yearList.length / 2)
        }
        cal.pickerMode = !cal.pickerMode
    }

    // Monday-first grid: JS getDay() is Sunday-based, so rotate it.
    readonly property int leadingBlanks: {
        var jsDay = new Date(viewYear, viewMonth, 1).getDay()
        return (jsDay + 6) % 7
    }
    readonly property int daysInMonth: new Date(viewYear, viewMonth + 1, 0).getDate()
    readonly property string todayIso: Qt.formatDate(new Date(), "yyyy-MM-dd")

    onClosed: {
        cal.pickerMode = false
        if (cal.pendingIso !== "") {
            var applied = cal.pendingIso
            cal.pendingIso = ""
            cal.accepted(applied)
        }
    }

    contentItem: Item {

        Column {
            id: body
            // Anchored on three sides only, so implicitHeight stays driven by
            // the children and can size the popup above.
            anchors { top: parent.top; left: parent.left; right: parent.right; margins: 12 }
            spacing: 8

            // ── Month header ──
            Item {
                width: parent.width
                height: 38

                Rectangle {
                    width: 38; height: 38; radius: 6
                    anchors.left: parent.left
                    visible: !cal.pickerMode
                    color: prevMA.pressed ? cal.cCellDown : cal.cCell
                    border.color: cal.cBorder; border.width: 1
                    Text {
                        anchors.centerIn: parent; text: "‹"
                        color: cal.cText; font.pixelSize: 22; font.bold: true
                    }
                    MouseArea { id: prevMA; anchors.fill: parent; onClicked: cal.shiftMonth(-1) }
                }

                // Tapping the title opens the month/year wheels.
                Rectangle {
                    anchors.centerIn: parent
                    width: titleText.implicitWidth + 34
                    height: 38
                    radius: 6
                    color: titleMA.pressed ? cal.cCellDown : "transparent"
                    border.color: cal.pickerMode ? cal.cAccent : "transparent"
                    border.width: 1

                    Text {
                        id: titleText
                        anchors.centerIn: parent
                        anchors.horizontalCenterOffset: -7
                        text: cal.monthNames[cal.viewMonth] + " " + cal.viewYear
                        color: cal.pickerMode ? cal.cAccent : cal.cText
                        font.pixelSize: 16; font.bold: true
                    }
                    Text {
                        anchors.right: parent.right
                        anchors.rightMargin: 9
                        anchors.verticalCenter: parent.verticalCenter
                        text: cal.pickerMode ? "▴" : "▾"
                        color: cal.pickerMode ? cal.cAccent : cal.cDim
                        font.pixelSize: 12
                    }
                    MouseArea { id: titleMA; anchors.fill: parent; onClicked: cal.togglePicker() }
                }

                Rectangle {
                    width: 38; height: 38; radius: 6
                    anchors.right: parent.right
                    visible: !cal.pickerMode
                    color: nextMA.pressed ? cal.cCellDown : cal.cCell
                    border.color: cal.cBorder; border.width: 1
                    Text {
                        anchors.centerIn: parent; text: "›"
                        color: cal.cText; font.pixelSize: 22; font.bold: true
                    }
                    MouseArea { id: nextMA; anchors.fill: parent; onClicked: cal.shiftMonth(1) }
                }
            }

            // ── Weekday header ──
            Row {
                width: parent.width
                spacing: 4
                visible: !cal.pickerMode
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
                visible: !cal.pickerMode

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
                            radius: 6
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

            // ── Month / year wheels ──
            // Same height as the day grid so opening them does not resize the
            // popup under the finger.
            Row {
                id: wheelRow
                width: parent.width
                height: 6 * dayGrid.cellH + 5 * dayGrid.spacing
                spacing: 8
                visible: cal.pickerMode

                Rectangle {
                    width: (parent.width - parent.spacing) * 0.6
                    height: parent.height
                    radius: 6
                    color: cal.cCell
                    border.color: cal.cBorder
                    border.width: 1

                    Tumbler {
                        id: monthWheel
                        anchors.fill: parent
                        anchors.margins: 4
                        model: cal.monthNames
                        visibleItemCount: 5
                        onCurrentIndexChanged: if (cal.pickerMode) cal.viewMonth = currentIndex
                        delegate: Text {
                            text: modelData
                            color: monthWheel.currentIndex === index ? cal.cAccent : cal.cText
                            font.pixelSize: monthWheel.currentIndex === index ? 17 : 14
                            font.bold: monthWheel.currentIndex === index
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            opacity: 1.0 - Math.abs(Tumbler.displacement) / 2.4
                        }
                    }
                }

                Rectangle {
                    width: (parent.width - parent.spacing) * 0.4
                    height: parent.height
                    radius: 6
                    color: cal.cCell
                    border.color: cal.cBorder
                    border.width: 1

                    Tumbler {
                        id: yearWheel
                        anchors.fill: parent
                        anchors.margins: 4
                        model: cal.yearList
                        visibleItemCount: 5
                        onCurrentIndexChanged: {
                            if (cal.pickerMode && currentIndex >= 0)
                                cal.viewYear = cal.yearList[currentIndex]
                        }
                        delegate: Text {
                            text: modelData
                            color: yearWheel.currentIndex === index ? cal.cAccent : cal.cText
                            font.pixelSize: yearWheel.currentIndex === index ? 17 : 14
                            font.bold: yearWheel.currentIndex === index
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            opacity: 1.0 - Math.abs(Tumbler.displacement) / 2.4
                        }
                    }
                }
            }

            // ── Bottom action ──
            Rectangle {
                width: parent.width
                height: 36
                radius: 6
                color: bottomMA.pressed ? cal.cCellDown : cal.cCell
                border.color: cal.cBorder; border.width: 1
                Text {
                    anchors.centerIn: parent
                    text: cal.pickerMode ? "DONE" : "TODAY"
                    color: cal.cText; font.pixelSize: 13; font.bold: true
                    font.letterSpacing: 1.1
                }
                MouseArea {
                    id: bottomMA
                    anchors.fill: parent
                    onClicked: {
                        if (cal.pickerMode) {
                            cal.pickerMode = false
                            return
                        }
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
