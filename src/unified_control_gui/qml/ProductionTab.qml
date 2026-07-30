// ─────────────────────────────────────────────────────────────────────────────
// ProductionTab.qml — PAGE 5: Production Output statistics (v2 redesign)
// Fetches data from fill_hp_web API (http://127.0.0.1:8080)
// ─────────────────────────────────────────────────────────────────────────────
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: prodTab
    property Item focusHost: null

    // ── Theme (CartridgePage palette — bolder) ──
    readonly property color cBg:       "transparent"
    readonly property color cPanel:    "#b30d1527"
    readonly property color cCardBg:   "#b30d1527"
    readonly property color cBorder:   "#1affffff"
    readonly property color cText:     "#c7dcef"
    readonly property color cMuted:    "#9fb3c8"
    readonly property color cOk:       "#3ed0b4"
    readonly property color cBad:      "#f0735c"
    readonly property color cWarn:     "#f5a623"
    readonly property color cCyan:     "#1f86e0"
    readonly property color cBlue:     "#1f86e0"
    readonly property color cPurple:   "#1f86e0"
    readonly property string monoFont: "JetBrains Mono, DejaVu Sans Mono, Consolas, monospace"

    readonly property string apiBase: "http://172.16.11.31:8080"
 
    // ── Data ──
    property var todayData: ({count:0, total_volume_ml:0, ok:0, ng:0, items:[]})
    property real todayRuntime: 0
    property string todayDate: Qt.formatDate(new Date(), "yyyy-MM-dd")
 
    property var dateData: ({count:0, total_volume_ml:0, ok:0, ng:0, items:[]})
    property real dateRuntime: 0
    property var rangeDays: []
 
    property var inkData: ({batches:0, total_g:0, items:[]})
 
    // ── Sub-tab navigation ──
    property int activeSection: 0   // 0=today, 1=byDate, 2=ink
 
    // ── API ──
    function apiGet(path, callback) {
        var xhr = new XMLHttpRequest()
        xhr.onreadystatechange = function() {
            if (xhr.readyState === XMLHttpRequest.DONE) {
                if (xhr.status === 200) {
                    try { callback(JSON.parse(xhr.responseText)) }
                    catch(e) { console.warn("ProductionTab parse:", e) }
                } else {
                    console.warn("ProductionTab API:", xhr.status, path)
                }
            }
        }
        xhr.open("GET", apiBase + path)
        xhr.send()
    }
 
    function convertDateFormat(val) {
        if (!val) return ""
        var cleaned = val.trim().replace(/\s+/g, "")
        var parts = cleaned.split("/")
        if (parts.length === 3) {
            var day = parts[0]
            var month = parts[1]
            var year = parts[2]
            if (day.length === 2 && month.length === 2 && year.length === 4) {
                return year + "-" + month + "-" + day
            }
        }
        return ""
    }
 
    function loadToday() {
        apiGet("/logs/today", function(d) {
            todayData = d
            todayDate = d.date || Qt.formatDate(new Date(), "yyyy-MM-dd")
        })
        apiGet("/runtime/today", function(d) { todayRuntime = d.total_minutes || 0 })
    }
    function loadDate() {
        var s = convertDateFormat(dateFromInput.text)
        var e = convertDateFormat(dateToInput.text)
        if (!s) return
        if (!e || e === s) {
            // Single day
            apiGet("/logs/date?date=" + encodeURIComponent(s), function(r) { dateData = r; rangeDays = [] })
            apiGet("/runtime/date?date=" + encodeURIComponent(s), function(r) { dateRuntime = r.total_minutes || 0 })
        } else {
            // Range
            apiGet("/logs/range?start=" + encodeURIComponent(s) + "&end=" + encodeURIComponent(e), function(r) {
                dateData = {count: r.count || 0, total_volume_ml: r.total_volume_ml || 0, ok: r.ok || 0, ng: r.ng || 0, items: []}
                dateRuntime = 0
                rangeDays = r.days || []
            })
        }
    }
    function loadInk() {
        var d = convertDateFormat(inkDateInput.text)
        if (!d) return
        var code = inkCodeInput.text.trim()
        var url = "/ink/date?date=" + encodeURIComponent(d)
        if (code) url += "&code=" + encodeURIComponent(code)
        apiGet(url, function(r) { inkData = r })
    }
 
    onVisibleChanged: if (visible) { loadToday(); loadInk() }
    Component.onCompleted: {
        var today = Qt.formatDate(new Date(), "dd/MM/yyyy")
        dateFromInput.text = today
        dateToInput.text = today
        inkDateInput.text = today
    }
 
    Rectangle { anchors.fill: parent; color: cBg }
 
    // ═══════════════════════════════════════════════════════════════════
    // HEADER
    // ═══════════════════════════════════════════════════════════════════
    Rectangle {
        id: prodHeader
        anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
        height: 56; color: cBg
        RowLayout {
            anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 10
            Text {
                text: "📊  PRODUCTION & RUNTIME"
                color: cText; font.pixelSize: 22; font.bold: true; font.letterSpacing: 1
            }
            Item { Layout.fillWidth: true }
            // Reload button
            Rectangle {
                Layout.preferredWidth: 110; Layout.preferredHeight: 36; radius: 8
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "#1f9e86" }
                    GradientStop { position: 1.0; color: "#1f9e86" }
                }
                border.color: cOk; border.width: 1
                Text { anchors.centerIn: parent; text: "🔄  Reload"; color: "#ffffff"; font.pixelSize: 15; font.bold: true }
                MotionMouseArea { anchors.fill: parent; onClicked: { loadToday(); loadDate(); loadInk() } }
            }
        }
        Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: cBorder }
    }
 
    // ═══════════════════════════════════════════════════════════════════
    // SUB-TAB BAR
    // ═══════════════════════════════════════════════════════════════════
    Rectangle {
        id: subTabBar
        anchors.top: prodHeader.bottom; anchors.left: parent.left; anchors.right: parent.right
        height: 46; color: cBg
 
        Row {
            anchors.verticalCenter: parent.verticalCenter; anchors.left: parent.left; anchors.leftMargin: 12
            spacing: 6
            Repeater {
                model: [
                    {idx: 0, lbl: "📋  Today"},
                    {idx: 1, lbl: "📅  By Date"},
                    {idx: 2, lbl: "🧪  Ink Batch"}
                ]
                Rectangle {
                    width: stLbl.implicitWidth + 32; height: 34; radius: 8
                    color: prodTab.activeSection === modelData.idx ? "#e61b2050" : "transparent"
                    border.color: prodTab.activeSection === modelData.idx ? cCyan : Qt.rgba(1,1,1,0.12)
                    border.width: prodTab.activeSection === modelData.idx ? 2 : 1
                    Text {
                        id: stLbl; anchors.centerIn: parent
                        text: modelData.lbl
                        color: prodTab.activeSection === modelData.idx ? cCyan : cMuted
                        font.pixelSize: 15; font.bold: prodTab.activeSection === modelData.idx
                    }
                    MotionMouseArea { anchors.fill: parent; onClicked: prodTab.activeSection = modelData.idx }
                }
            }
        }
        Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: cBorder }
    }

    // ═══════════════════════════════════════════════════════════════════
    // CONTENT AREA (stacked by activeSection)
    // ═══════════════════════════════════════════════════════════════════
    Flickable {
        id: contentFlick
        anchors.top: subTabBar.bottom; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom
        anchors.margins: 16
        contentHeight: contentCol.childrenRect.height + contentCol.y + 12; clip: true
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

        ColumnLayout {
            id: contentCol; width: prodTab.width - 32; spacing: 16

            // ──────────────────────────────────────────────
            // SECTION 0: TODAY
            // ──────────────────────────────────────────────
            ColumnLayout {
                visible: activeSection === 0
                Layout.fillWidth: true; spacing: 14

                Text {
                    text: "TODAY  —  " + prodTab.todayDate
                    color: cCyan; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                }

                // Stat row
                RowLayout {
                    Layout.fillWidth: true; spacing: 10
                    StatCard { Layout.fillWidth: true; num: todayData.count;           lbl: "Filled Batches";   accent: cBlue }
                    StatCard { Layout.fillWidth: true; num: todayData.total_volume_ml; lbl: "Total Volume (ml)"; accent: cPurple }
                    StatCard { Layout.fillWidth: true; num: todayData.ok;              lbl: "G";               accent: cOk }
                    StatCard { Layout.fillWidth: true; num: todayData.ng;              lbl: "NG";               accent: cBad }
                    StatCard { Layout.fillWidth: true; num: todayRuntime.toFixed(1);   lbl: "Runtime (min)";    accent: cWarn }
                }

                // Table
                DataTable {
                    Layout.fillWidth: true
                    headers: ["No.", "Time", "Machine", "Volume (ml)", "Result"]
                    colWidths: [0.8, 2.0, 1.5, 2.0, 1.2]
                    rows: buildFillRows(todayData.items)
                }
            }

            // ──────────────────────────────────────────────
            // SECTION 1: BY DATE
            // ──────────────────────────────────────────────
            ColumnLayout {
                visible: activeSection === 1
                Layout.fillWidth: true; spacing: 14

                Text { text: "BY DATE"; color: cCyan; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }

                RowLayout {
                    spacing: 10
                    Text { text: "From Date:"; color: cText; font.pixelSize: 16 }
                    DateBox { id: dateFromInput }
                    Text { text: "To Date:"; color: cText; font.pixelSize: 16 }
                    DateBox { id: dateToInput }
                    ActionBtn { label: "View"; onClicked: loadDate() }
                }

                RowLayout {
                    Layout.fillWidth: true; spacing: 10
                    StatCard { Layout.fillWidth: true; num: dateData.count;           lbl: "Batches";          accent: cBlue }
                    StatCard { Layout.fillWidth: true; num: dateData.total_volume_ml; lbl: "Volume (ml)";      accent: cPurple }
                    StatCard { Layout.fillWidth: true; num: dateData.ok;              lbl: "OK";               accent: cOk }
                    StatCard { Layout.fillWidth: true; num: dateData.ng;              lbl: "NG";               accent: cBad }
                    StatCard { Layout.fillWidth: true; num: dateRuntime.toFixed(1);   lbl: "Runtime (min)";    accent: cWarn }
                }

                // Detail table (single day)
                DataTable {
                    visible: rangeDays.length === 0
                    Layout.fillWidth: true
                    headers: ["No.", "Time", "Machine", "Volume (ml)", "Result"]
                    colWidths: [0.8, 2.0, 1.5, 2.0, 1.2]
                    rows: buildFillRows(dateData.items)
                }

                // Range breakdown table (multi-day)
                DataTable {
                    visible: rangeDays.length > 0
                    Layout.fillWidth: true
                    headers: ["Date", "Batches Count", "Volume (ml)"]
                    colWidths: [2.0, 1.5, 2.0]
                    rows: {
                        var r = []
                        for (var i = 0; i < rangeDays.length; i++) {
                            var d = rangeDays[i]
                            r.push([d.date || "", String(d.count || 0), String(d.total_volume_ml || 0)])
                        }
                        return r
                    }
                }
            }

            // ──────────────────────────────────────────────
            // SECTION 2: INK BATCH
            // ──────────────────────────────────────────────
            ColumnLayout {
                visible: activeSection === 2
                Layout.fillWidth: true; spacing: 14

                Text { text: "INK BATCH BY DATE"; color: cCyan; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }

                RowLayout {
                    spacing: 10
                    Text { text: "Date:"; color: cText; font.pixelSize: 16 }
                    DateBox { id: inkDateInput }
                    Text { text: "Usage Code:"; color: cText; font.pixelSize: 16 }
                    Rectangle {
                        width: 180; height: 38; radius: 8
                        color: "#e61b2050"; border.color: cAccent; border.width: 1
                        SmartTextInput {
                            id: inkCodeInput
                            focusHost: prodTab.focusHost
                            anchors.fill: parent; anchors.margins: 8
                            color: cText; font.pixelSize: 15; font.family: prodTab.monoFont
                            clip: true; verticalAlignment: TextInput.AlignVCenter
                            Text {
                                visible: !parent.text && !parent.activeFocus
                                text: "All profiles"
                                color: cMuted; font.pixelSize: 14
                                anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                    }
                    ActionBtn { label: "View"; onClicked: loadInk() }
                }
 
                RowLayout {
                    Layout.fillWidth: true; spacing: 10
                    StatCard { Layout.fillWidth: true; num: inkData.batches;      lbl: "Ink Batches Count";  accent: cBlue }
                    StatCard { Layout.fillWidth: true; num: inkData.total_g || 0; lbl: "Total Weight (g)";   accent: cPurple }
                }
 
                DataTable {
                    Layout.fillWidth: true
                    headers: ["Time", "Operator", "Usage Code", "Ink Name",
                              "Lot PI", "Lot CI", "Density", "Mode", "Volume (ml)",
                              "Chamber Pressure", "8-Cartridge Pressures", "g Used", "g Left"]
                    colWidths: [1.3, 1.0, 1.0, 1.0, 1.2, 0.8, 0.8, 0.9, 0.9, 0.9, 2.8, 0.8, 0.8]
                    rows: {
                        var r = []
                        var items = inkData.items || []
                        for (var i = 0; i < items.length; i++) {
                            var it = items[i]
                            var cps = (it.cart_pressures || [])
                            var cpStr = ""
                            for (var j = 0; j < cps.length; j++) {
                                if (j > 0) cpStr += "/"
                                cpStr += Math.round(cps[j])
                            }
                            var chamberP = it.chamber_pressure ? Math.round(it.chamber_pressure) : ""
                            r.push([
                                it.time || "", it.operator || "", it.scan_code || "", it.code || "",
                                it.lot_pi || "", it.lot_ci || "",
                                it.density_g_ml !== undefined ? String(it.density_g_ml) : "",
                                it.mode || "",
                                it.volume_ml !== undefined ? String(it.volume_ml) : "",
                                String(chamberP), cpStr,
                                it.gram_used !== undefined ? String(it.gram_used) : "",
                                it.gram_remaining !== undefined ? String(it.gram_remaining) : ""
                            ])
                        }
                        return r
                    }
                }
            }
 
        } // contentCol
    } // Flickable
 
    // ── Helper JS ──
    function buildFillRows(items) {
        var r = []
        items = items || []
        for (var i = 0; i < items.length; i++) {
            var it = items[i]
            r.push([it.seq || "", it.time || "", it.machine || "",
                     it.volume_ml !== undefined ? String(it.volume_ml) : "",
                     it.result || ""])
        }
        return r
    }
 
    // ═══════════════════════════════════════════════════════════════════
    // INLINE COMPONENTS
    // ═══════════════════════════════════════════════════════════════════
 
    // ── Stat Card: big number + label ──
    component StatCard: Rectangle {
        property var num: 0
        property string lbl: ""
        property color accent: cText
        implicitHeight: 90; radius: 10
        color: cCard
        border.color: cBorder; border.width: 1
 
        // Accent glow line at top
        Rectangle {
            anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
            anchors.leftMargin: 12; anchors.rightMargin: 12
            height: 3; radius: 2; color: accent; opacity: 0.6
        }
 
        ColumnLayout {
            anchors.centerIn: parent; spacing: 4
            Text {
                text: String(num)
                color: accent; font.pixelSize: 34; font.bold: true; font.family: prodTab.monoFont
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: lbl; color: cMuted; font.pixelSize: 14; font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
        }
    }
 
    // ── Date input box ──
    component DateBox: Rectangle {
        property alias text: dateField.text
        width: 160; height: 38; radius: 8
        color: "#e61b2050"; border.color: cAccent; border.width: 1
        SmartTextInput {
            id: dateField
            focusHost: prodTab.focusHost
            anchors.fill: parent; anchors.margins: 8
            color: cText; font.pixelSize: 15; font.family: prodTab.monoFont
            clip: true; verticalAlignment: TextInput.AlignVCenter
            inputMask: "99/99/9999; "
            inputMethodHints: Qt.ImhDigitsOnly
            Text {
                visible: (dateField.text.trim() === "//" || dateField.text === "  /  /    ") && !dateField.activeFocus
                text: "DD/MM/YYYY"; color: cMuted; font.pixelSize: 14
                anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter
            }
        }
    }
 
    // ── Action button ──
    component ActionBtn: Rectangle {
        id: actionBtn
        property string label: "Xem"
        signal clicked()
        width: 80; height: 38; radius: 8
        gradient: Gradient {
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: abMA.pressed ? Qt.darker("#1f86e0", 1.2) : "#1f86e0" }
            GradientStop { position: 1.0; color: abMA.pressed ? Qt.darker("#163a52", 1.2) : "#163a52" }
        }
        border.color: "#67d0ff"; border.width: 1
        Text { anchors.centerIn: parent; text: actionBtn.label; color: "#ffffff"; font.pixelSize: 16; font.bold: true }
        MotionMouseArea { id: abMA; anchors.fill: parent; onClicked: actionBtn.clicked() }
    }
 
    // ── Data table with headers + rows ──
    component DataTable: Rectangle {
        id: tableRoot
        property var headers: []
        property var colWidths: [] // weights
        property var rows: []
 
        color: "transparent"
        border.color: cBorder
        border.width: 1
        radius: 6
        clip: true
 
        readonly property real totalWeight: {
            var sum = 0
            for (var i = 0; i < colWidths.length; i++) {
                sum += colWidths[i]
            }
            return sum > 0 ? sum : 1
        }
 
        function getCellWidth(index, totalWidth) {
            var w = colWidths.length > index ? colWidths[index] : 1
            return (totalWidth * w / totalWeight)
        }
 
        implicitHeight: tblCol.implicitHeight
 
        ColumnLayout {
            id: tblCol
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 0
 
            // Header Row
            Rectangle {
                Layout.fillWidth: true
                implicitHeight: 40
                color: cCard
 
                Row {
                    anchors.fill: parent
                    spacing: 0
                    Repeater {
                        model: headers
                        Rectangle {
                            width: tableRoot.getCellWidth(index, parent.width)
                            height: parent.height
                            color: "transparent"
                            // Draw border on the right (except last item)
                            Rectangle {
                                anchors.top: parent.top
                                anchors.bottom: parent.bottom
                                anchors.right: parent.right
                                width: index < headers.length - 1 ? 1 : 0
                                color: cBorder
                            }
                            Text {
                                anchors.fill: parent
                                anchors.margins: 4
                                text: modelData
                                color: cCyan
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                elide: Text.ElideRight
                            }
                        }
                    }
                }
                
                // Draw bottom border under header
                Rectangle {
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom
                    height: 1
                    color: cBorder
                }
            }

            // Empty State
            Rectangle {
                visible: rows.length === 0
                Layout.fillWidth: true
                implicitHeight: 50
                color: "transparent"
                Text {
                    anchors.centerIn: parent
                    text: "— No Data Available —"
                    color: cMuted
                    font.pixelSize: 16
                    font.italic: true
                }
            }

            // Data rows
            Repeater {
                model: rows.length
                Rectangle {
                    id: rowRect
                    Layout.fillWidth: true
                    implicitHeight: 36
                    color: index % 2 === 0 ? "transparent" : Qt.rgba(1,1,1,0.03)
                    
                    readonly property int rowIndex: index

                    Row {
                        anchors.fill: parent
                        spacing: 0
                        Repeater {
                            model: rows[rowRect.rowIndex]
                            Rectangle {
                                width: tableRoot.getCellWidth(index, parent.width)
                                height: parent.height
                                color: "transparent"
                                
                                // Draw border on the right (except last item)
                                Rectangle {
                                    anchors.top: parent.top
                                    anchors.bottom: parent.bottom
                                    anchors.right: parent.right
                                    width: index < headers.length - 1 ? 1 : 0
                                    color: cBorder
                                }

                                Text {
                                    anchors.fill: parent
                                    anchors.margins: 4
                                    text: modelData
                                    color: {
                                        var hdr = headers.length > index ? headers[index] : ""
                                        if (hdr === "Result" || hdr === "Kết quả") {
                                            return String(modelData).toUpperCase() === "NG" ? cBad : cOk
                                        }
                                        return cText
                                    }
                                    font.pixelSize: 13
                                    font.family: prodTab.monoFont
                                    font.bold: {
                                        var hdr = headers.length > index ? headers[index] : ""
                                        return hdr === "Result" || hdr === "Kết quả"
                                    }
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    elide: Text.ElideRight
                                }
                            }
                        }
                    }

                    // Draw bottom border under each row
                    Rectangle {
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        height: 1
                        color: cBorder
                    }
                }
            }
        }
    }
}
