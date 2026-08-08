// ─────────────────────────────────────────────────────────────────────────────
// ProductionTab.qml — Production Output synchronized with RevPi A
// logs.html reads these same live JSON APIs; there is no static JSON snapshot.
// ─────────────────────────────────────────────────────────────────────────────
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtGraphicalEffects 1.15

Item {
    id: prodTab
    property Item focusHost: null

    // Reuse the exact Control Dashboard glass palette. Fallbacks keep this
    // component valid when opened by a standalone QML preview.
    readonly property color cBg:       "transparent"
    readonly property color cPanel:    focusHost ? focusHost.cControlPanel : "#990d1e32"
    readonly property color cCardBg:   focusHost ? focusHost.cControlCard : "#8806101d"
    readonly property color cBorder:   focusHost ? focusHost.cControlBorder : "#1affffff"
    readonly property color cHover:    focusHost ? focusHost.cHover : "#40ffffff"
    readonly property color cText:     focusHost ? focusHost.cText : "#c7dcef"
    readonly property color cTitle:    focusHost ? focusHost.cCardTitle : "#ffffff"
    readonly property color cSection:  focusHost ? focusHost.cFunctionLabelText : "#bfe0f5"
    readonly property color cMuted:    focusHost ? focusHost.cDim : "#74899f"
    readonly property color cOk:       "#3ed0b4"
    readonly property color cBad:      focusHost ? focusHost.cRed : "#f0735c"
    readonly property color cBadBg:    Qt.rgba(cBad.r, cBad.g, cBad.b, 0.15)
    readonly property color cWarn:     focusHost ? focusHost.cOrange : "#f5a623"
    readonly property color cCyan:     focusHost ? focusHost.cCyan : "#36b6ff"
    readonly property color cAccent:   focusHost ? focusHost.cAccent : "#7fcdf5"
    readonly property color cFieldStart: focusHost ? focusHost.cFunctionFieldStart : "#d10f3042"
    readonly property color cFieldEnd: focusHost ? focusHost.cFunctionFieldEnd : "#d10a212f"
    readonly property color cFieldBorder: focusHost ? focusHost.cFunctionFieldBorder : "#4d67d0ff"
    readonly property color cFieldText: focusHost ? focusHost.cFunctionFieldText : "#d6f1ff"
    readonly property color cActionStart: focusHost ? focusHost.cDashboardActionStart : "#163a52"
    readonly property color cActionEnd: focusHost ? focusHost.cDashboardActionEnd : "#081627"
    readonly property color cActionBorder: focusHost ? focusHost.cDashboardActionBorder : "#163a52"
    readonly property color cBaseStart: focusHost ? focusHost.cBtnBaseStart : "#0c1726"
    readonly property color cBaseEnd: focusHost ? focusHost.cBtnBaseEnd : "#06101d"
    readonly property color cBaseBorder: focusHost ? focusHost.cBtnBaseBorder : "#163a52"
    readonly property color cTabTop: focusHost ? focusHost.cTabSelectedTop : "#9ee0f2ff"
    readonly property color cTabMid: focusHost ? focusHost.cTabSelectedMid : "#8cd1e8fc"
    readonly property color cTabBottom: focusHost ? focusHost.cTabSelectedBottom : "#7ab8d9f5"
    readonly property color cTabText: focusHost ? focusHost.cTabSelectedContent : "#06101d"
    // Match Technical System typography: its labels/values use the app default family.
    readonly property string dashboardTextFamily: Qt.application.font.family

    readonly property string apiHost: (typeof revpiAHost !== "undefined" && revpiAHost)
                                      ? String(revpiAHost) : "172.16.11.31"
    readonly property string apiBase: "http://" + apiHost + ":8090"

    property int pendingRequests: 0
    property bool apiOnline: false
    property string apiError: ""
    property string lastSync: ""
    property bool initialized: false
    property bool initializing: false
    property var activeApiRequests: []
 
    // ── Data ──
    property var todayData: ({count:0, total_volume_ml:0, ok:0, ng:0, items:[]})
    property real todayRuntime: 0
    property string todayDate: Qt.formatDate(new Date(), "yyyy-MM-dd")
 
    property var dateData: ({count:0, total_volume_ml:0, ok:0, ng:0, items:[]})
    property real dateRuntime: 0
    property var rangeDays: []
 
    property var inkData: ({batches:0, total_g:0, items:[]})
    property var alertsData: ({count:0, opened:0, resolved:0, items:[]})
    property var eventsData: ({count:0, error:0, warn:0, items:[]})
 
    // ── Sub-tab navigation ──
    property int activeSection: 0   // 0=today, 1=byDate, 2=ink, 3=alerts, 4=events
 
    // ── API ──
    function expireApiRequests() {
        var now = Date.now()
        var requests = activeApiRequests.slice(0)
        for (var i = 0; i < requests.length; i++) {
            if (requests[i].deadline <= now)
                requests[i].expire()
        }
    }

    function apiGet(path, callback, errorCallback) {
        var xhr = new XMLHttpRequest()
        var completed = false
        var requestToken = null

        function finishRequest() {
            if (completed)
                return false
            completed = true
            var tokenIndex = activeApiRequests.indexOf(requestToken)
            if (tokenIndex >= 0)
                activeApiRequests.splice(tokenIndex, 1)
            pendingRequests = Math.max(0, pendingRequests - 1)
            return true
        }

        function failRequest(message) {
            if (!finishRequest())
                return
            apiOnline = false
            if (!apiError)
                apiError = message + "  (RevPi A " + apiHost + ":8090)"
            console.warn("ProductionTab:", message, path)
            if (errorCallback)
                errorCallback(message)
        }

        pendingRequests += 1
        requestToken = {
            deadline: Date.now() + 6000,
            expire: function() {
                failRequest("API không phản hồi sau 6 giây")
                xhr.abort()
            }
        }
        activeApiRequests.push(requestToken)
        xhr.onreadystatechange = function() {
            if (xhr.readyState !== XMLHttpRequest.DONE || completed)
                return

            if (xhr.status === 200) {
                var parsed
                try {
                    parsed = JSON.parse(xhr.responseText)
                } catch (e) {
                    failRequest("Dữ liệu JSON không hợp lệ")
                    return
                }
                if (!finishRequest())
                    return
                if (!apiError)
                    apiOnline = true
                lastSync = Qt.formatTime(new Date(), "HH:mm:ss")
                callback(parsed)
            } else {
                failRequest("API trả về HTTP " + xhr.status)
            }
        }
        xhr.onerror = function() { failRequest("Không kết nối được API Production Output") }
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

    function isoToDisplay(isoDate) {
        var parts = String(isoDate || "").split("-")
        if (parts.length !== 3)
            return ""
        return parts[2] + "/" + parts[1] + "/" + parts[0]
    }


    function setAllDates(isoDate) {
        var displayDate = isoToDisplay(isoDate)
        if (!displayDate)
            displayDate = Qt.formatDate(new Date(), "dd/MM/yyyy")
        dateFromInput.text = displayDate
        dateToInput.text = displayDate
        inkDateInput.text = displayDate
        alertsDateInput.text = displayDate
        eventsDateInput.text = displayDate
    }

    function initializeFromServer() {
        if (initialized || initializing)
            return
        initializing = true
        apiError = ""
        apiGet("/api/time", function(d) {
            initializing = false
            initialized = true
            setAllDates(d.local_date || Qt.formatDate(new Date(), "yyyy-MM-dd"))
            loadAllData()
        }, function() {
            initializing = false
            initialized = true
            setAllDates(Qt.formatDate(new Date(), "yyyy-MM-dd"))
            loadAllData()
        })
    }

    function loadAllData(clearError) {
        if (clearError === undefined || clearError)
            apiError = ""
        apiOnline = false
        loadToday()
        loadDate()
        loadInk()
        loadAlerts()
        loadEvents()
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

    function loadAlerts() {
        var d = convertDateFormat(alertsDateInput.text)
        if (!d) return
        apiGet("/alerts/date?date=" + encodeURIComponent(d), function(r) { alertsData = r })
    }

    function loadEvents() {
        var d = convertDateFormat(eventsDateInput.text)
        if (!d) return
        apiGet("/events/date?date=" + encodeURIComponent(d), function(r) { eventsData = r })
    }

    onVisibleChanged: {
        if (visible) {
            if (initialized)
                loadAllData()
            else
                initializeFromServer()
        }
    }
    Component.onCompleted: {
        // Keep date fields ready, but do not open the API requests while the
        // Production Output page is still hidden inside CartridgePage.
        setAllDates(Qt.formatDate(new Date(), "yyyy-MM-dd"))
        if (visible)
            initializeFromServer()
    }

    Rectangle { anchors.fill: parent; color: cBg }

    Timer {
        interval: 500
        repeat: true
        running: pendingRequests > 0
        onTriggered: expireApiRequests()
    }

    // ═══════════════════════════════════════════════════════════════════
    // HEADER
    // ═══════════════════════════════════════════════════════════════════
    Rectangle {
        id: prodHeader
        anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
        anchors.topMargin: 10; anchors.leftMargin: 10; anchors.rightMargin: 10
        height: 58; color: cPanel; radius: 6
        border.color: cBorder; border.width: 1
        GlassHighlight {}
        RowLayout {
            anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 10

            Item {
                Layout.preferredWidth: 24; Layout.preferredHeight: 24
                Image {
                    id: productionHeaderIcon
                    anchors.fill: parent; source: "qrc:/qml/icons/database_search.svg"
                    fillMode: Image.PreserveAspectFit; smooth: true; visible: false
                }
                ColorOverlay { anchors.fill: productionHeaderIcon; source: productionHeaderIcon; color: cCyan }
            }
            Text {
                text: "PRODUCTION & RUNTIME"
                color: cTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
            }
            Item { Layout.fillWidth: true }

            BusyIndicator {
                Layout.preferredWidth: 28; Layout.preferredHeight: 28
                running: pendingRequests > 0
                visible: running
            }
            Rectangle {
                Layout.preferredWidth: 9; Layout.preferredHeight: 9; radius: 5
                color: apiError ? cBad : (apiOnline ? cOk : cMuted)
            }
            Text {
                text: pendingRequests > 0
                      ? "Synchronizing RevPi A…"
                      : (apiError ? "RevPi A offline" : (lastSync ? "Synced " + lastSync : "Waiting for RevPi A"))
                color: apiError ? cBad : (apiOnline ? cOk : cMuted)
                font.pixelSize: 13; font.bold: true
            }

            // Reload button
            Rectangle {
                Layout.preferredWidth: 110; Layout.preferredHeight: 44; radius: 8
                enabled: pendingRequests === 0
                opacity: enabled ? 1.0 : 0.48
                gradient: Gradient {
                    orientation: Gradient.Horizontal
                    GradientStop { position: 0.0; color: reloadMA.pressed ? Qt.darker(cActionStart, 1.15) : cActionStart }
                    GradientStop { position: 1.0; color: reloadMA.pressed ? Qt.darker(cActionEnd, 1.15) : cActionEnd }
                }
                border.color: cActionBorder; border.width: 1
                Row {
                    anchors.centerIn: parent; spacing: 7
                    Item {
                        width: 17; height: 17
                        Image {
                            id: reloadIcon
                            anchors.fill: parent; source: "qrc:/qml/icons/refresh_cw.svg"
                            fillMode: Image.PreserveAspectFit; smooth: true; visible: false
                        }
                        ColorOverlay { anchors.fill: reloadIcon; source: reloadIcon; color: cTitle }
                    }
                    Text { text: "Reload"; color: cTitle; font.pixelSize: 14; font.bold: true }
                }
                MotionMouseArea { id: reloadMA; anchors.fill: parent; onClicked: loadAllData() }
            }
        }
    }

    Rectangle {
        id: apiErrorBanner
        anchors.top: prodHeader.bottom; anchors.left: parent.left; anchors.right: parent.right
        anchors.topMargin: 6; anchors.leftMargin: 10; anchors.rightMargin: 10
        height: visible ? 54 : 0
        visible: apiError.length > 0
        color: cBadBg
        border.color: cBad; border.width: 1; radius: 6

        RowLayout {
            anchors.fill: parent; anchors.leftMargin: 14; anchors.rightMargin: 8; spacing: 12
            Text {
                text: "Production Output chưa thể đồng bộ: " + apiError
                color: cTitle; font.pixelSize: 14; font.bold: true
                Layout.fillWidth: true; wrapMode: Text.Wrap
            }
            ActionBtn {
                label: "Retry"
                Layout.preferredWidth: 88
                onClicked: loadAllData()
            }
        }
    }
 
    // ═══════════════════════════════════════════════════════════════════
    // SUB-TAB BAR
    // ═══════════════════════════════════════════════════════════════════
    Rectangle {
        id: subTabBar
        anchors.top: apiErrorBanner.bottom; anchors.left: parent.left; anchors.right: parent.right
        anchors.topMargin: 6; anchors.leftMargin: 10; anchors.rightMargin: 10
        height: 50; color: cPanel; radius: 6
        border.color: cBorder; border.width: 1
        GlassHighlight {}
 
        Row {
            anchors.verticalCenter: parent.verticalCenter; anchors.horizontalCenter: parent.horizontalCenter
            spacing: 6
            Repeater {
                model: [
                    {idx: 0, lbl: "Today", icon: "qrc:/qml/icons/schedule.svg"},
                    {idx: 1, lbl: "By Date", icon: "qrc:/qml/icons/database_search.svg"},
                    {idx: 2, lbl: "Ink Batch", icon: "qrc:/qml/icons/droplet.svg"},
                    {idx: 3, lbl: "Operator Alerts", icon: "qrc:/qml/icons/message_circle_warning.svg"},
                    {idx: 4, lbl: "Stop Errors", icon: "qrc:/qml/icons/octagon_x_lucide.svg"}
                ]
                Rectangle {
                    id: subTabButton
                    readonly property bool selected: prodTab.activeSection === modelData.idx
                    readonly property int iconSlot: 17
                    readonly property int iconGap: 7
                    // Reserve the icon slot on BOTH sides so the label sits on the
                    // pill centre instead of being pushed right by the icon.
                    width: subTabLabel.implicitWidth + 2 * (iconSlot + iconGap) + 28
                    height: 38; radius: 8
                    color: "transparent"
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: subTabButton.selected ? cTabTop : "transparent" }
                        GradientStop { position: 0.54; color: subTabButton.selected ? cTabMid : "transparent" }
                        GradientStop { position: 1.0; color: subTabButton.selected ? cTabBottom : "transparent" }
                    }
                    border.color: subTabButton.selected ? cBorder : (subTabMA.containsMouse ? cHover : "transparent")
                    border.width: 1

                    Text {
                        id: subTabLabel
                        anchors.centerIn: parent
                        text: modelData.lbl
                        color: subTabButton.selected ? cTabText : cText
                        font.pixelSize: 14; font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    Item {
                        width: subTabButton.iconSlot; height: subTabButton.iconSlot
                        anchors.right: subTabLabel.left
                        anchors.rightMargin: subTabButton.iconGap
                        anchors.verticalCenter: parent.verticalCenter
                        Image {
                            id: subTabIcon
                            anchors.fill: parent; source: modelData.icon
                            fillMode: Image.PreserveAspectFit; smooth: true; visible: false
                        }
                        ColorOverlay {
                            anchors.fill: subTabIcon; source: subTabIcon
                            color: subTabButton.selected ? cTabText : cSection
                        }
                    }
                    MotionMouseArea {
                        id: subTabMA; anchors.fill: parent; motionEnabled: false
                        onClicked: prodTab.activeSection = modelData.idx
                    }
                }
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // CONTENT AREA (stacked by activeSection)
    // ═══════════════════════════════════════════════════════════════════
    Flickable {
        id: contentFlick
        anchors.top: subTabBar.bottom; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom
        anchors.topMargin: 8; anchors.leftMargin: 10; anchors.rightMargin: 10; anchors.bottomMargin: 10
        contentHeight: contentPanel.height + 10; clip: true
        // Vertical only, so a horizontal drag is left for the page swipe in
        // CartridgePage instead of being claimed by this Flickable.
        flickableDirection: Flickable.VerticalFlick
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

        Rectangle {
            id: contentPanel
            width: contentFlick.width
            height: contentCol.implicitHeight + 28
            color: cPanel; border.color: cBorder; border.width: 1; radius: 6
            GlassHighlight {}

            ColumnLayout {
                id: contentCol
                x: 14; y: 14; width: contentPanel.width - 28; spacing: 16

            // ──────────────────────────────────────────────
            // SECTION 0: TODAY
            // ──────────────────────────────────────────────
            ColumnLayout {
                visible: activeSection === 0
                Layout.fillWidth: true; spacing: 14

                Text {
                    text: "TODAY  —  " + prodTab.todayDate
                    color: cTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                }

                // Stat row
                RowLayout {
                    Layout.fillWidth: true; spacing: 10
                    StatCard { Layout.fillWidth: true; num: todayData.count;           lbl: "Filled Batches";   accent: cAccent }
                    StatCard { Layout.fillWidth: true; num: todayData.total_volume_ml; lbl: "Total Volume (ml)"; accent: cCyan }
                    StatCard { Layout.fillWidth: true; num: todayData.ok;              lbl: "OK";              accent: cOk }
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

                Text { text: "PRODUCTION BY DATE"; color: cTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }

                RowLayout {
                    spacing: 10
                    Text { text: "From Date:"; color: cSection; font.pixelSize: 15; font.bold: true }
                    DateBox { id: dateFromInput; onDateApplied: loadDate() }
                    Text { text: "To Date:"; color: cSection; font.pixelSize: 15; font.bold: true }
                    DateBox { id: dateToInput;   onDateApplied: loadDate() }
                    ActionBtn { label: "View"; onClicked: loadDate() }
                }

                RowLayout {
                    Layout.fillWidth: true; spacing: 10
                    StatCard { Layout.fillWidth: true; num: dateData.count;           lbl: "Batches";          accent: cAccent }
                    StatCard { Layout.fillWidth: true; num: dateData.total_volume_ml; lbl: "Volume (ml)";      accent: cCyan }
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

                Text { text: "INK BATCH BY DATE"; color: cTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }

                RowLayout {
                    spacing: 10
                    Text { text: "Date:"; color: cSection; font.pixelSize: 15; font.bold: true }
                    DateBox { id: inkDateInput;  onDateApplied: loadInk() }
                    Text { text: "Usage Code:"; color: cSection; font.pixelSize: 15; font.bold: true }
                    Rectangle {
                        width: 180; height: 44; radius: 8
                        color: "transparent"; border.color: cFieldBorder; border.width: 1
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: cFieldStart }
                            GradientStop { position: 1.0; color: cFieldEnd }
                        }
                        SmartTextInput {
                            id: inkCodeInput
                            focusHost: prodTab.focusHost
                            anchors.fill: parent; anchors.margins: 8
                            color: cFieldText; font.pixelSize: 15; font.family: prodTab.dashboardTextFamily
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
                    StatCard { Layout.fillWidth: true; num: inkData.batches;      lbl: "Ink Batches Count";  accent: cAccent }
                    StatCard { Layout.fillWidth: true; num: inkData.total_g || 0; lbl: "Total Weight (g)";   accent: cCyan }
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

            // ──────────────────────────────────────────────
            // SECTION 3: OPERATOR ALERTS
            // ──────────────────────────────────────────────
            ColumnLayout {
                visible: activeSection === 3
                Layout.fillWidth: true; spacing: 14

                Text { text: "OPERATOR ALERTS BY DATE"; color: cTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }

                RowLayout {
                    spacing: 10
                    Text { text: "Date:"; color: cSection; font.pixelSize: 15; font.bold: true }
                    DateBox { id: alertsDateInput; onDateApplied: loadAlerts() }
                    ActionBtn { label: "View"; onClicked: loadAlerts() }
                }

                RowLayout {
                    Layout.fillWidth: true; spacing: 10
                    StatCard { Layout.fillWidth: true; num: alertsData.count || 0;    lbl: "Alert Events"; accent: cAccent }
                    StatCard { Layout.fillWidth: true; num: alertsData.opened || 0;   lbl: "Opened";       accent: cWarn }
                    StatCard { Layout.fillWidth: true; num: alertsData.resolved || 0; lbl: "Resolved";     accent: cOk }
                }

                DataTable {
                    Layout.fillWidth: true
                    headers: ["Time", "Machine", "Operator", "Event", "Level", "Area",
                              "Mode", "Fill", "Dosing", "CR", "Detail", "Resolution"]
                    colWidths: [1.2, 0.9, 1.0, 0.9, 0.8, 0.9, 0.9, 0.8, 0.8, 0.8, 2.8, 2.2]
                    rows: buildAlertRows(alertsData.items)
                }
            }

            // ──────────────────────────────────────────────
            // SECTION 4: SYSTEM STOP ERRORS
            // ──────────────────────────────────────────────
            ColumnLayout {
                visible: activeSection === 4
                Layout.fillWidth: true; spacing: 14

                Text { text: "SYSTEM STOP ERRORS BY DATE"; color: cTitle; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }

                RowLayout {
                    spacing: 10
                    Text { text: "Date:"; color: cSection; font.pixelSize: 15; font.bold: true }
                    DateBox { id: eventsDateInput; onDateApplied: loadEvents() }
                    ActionBtn { label: "View"; onClicked: loadEvents() }
                }

                RowLayout {
                    Layout.fillWidth: true; spacing: 10
                    StatCard { Layout.fillWidth: true; num: eventsData.count || 0; lbl: "Stop Events"; accent: cAccent }
                    StatCard { Layout.fillWidth: true; num: eventsData.error || 0; lbl: "Errors";      accent: cBad }
                    StatCard { Layout.fillWidth: true; num: eventsData.warn || 0;  lbl: "Warnings";    accent: cWarn }
                }

                DataTable {
                    Layout.fillWidth: true
                    headers: ["Time", "Machine", "Operator", "Level", "Area", "Mode",
                              "Fill", "Dosing", "CR", "Detail", "Action"]
                    // Action carries the repair instruction and is the reason
                    // an operator opens this table, so it gets the widest
                    // share and wraps rather than eliding.
                    colWidths: [1.2, 0.9, 1.0, 0.8, 0.9, 0.9, 0.8, 0.8, 0.8, 2.2, 4.2]
                    wrapColumns: ["Action"]
                    rows: buildEventRows(eventsData.items)
                }
            }
 
            } // contentCol
        } // contentPanel
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

    function valueOrEmpty(value) {
        return value === undefined || value === null ? "" : String(value)
    }

    function buildAlertRows(items) {
        var r = []
        items = items || []
        for (var i = 0; i < items.length; i++) {
            var it = items[i]
            r.push([valueOrEmpty(it.time), valueOrEmpty(it.machine), valueOrEmpty(it.operator),
                    valueOrEmpty(it.event), valueOrEmpty(it.level), valueOrEmpty(it.area),
                    valueOrEmpty(it.mode), valueOrEmpty(it.fill_state), valueOrEmpty(it.dosing_state),
                    valueOrEmpty(it.cr_state), valueOrEmpty(it.message), valueOrEmpty(it.resolution)])
        }
        return r
    }

    function buildEventRows(items) {
        var r = []
        items = items || []
        for (var i = 0; i < items.length; i++) {
            var it = items[i]
            r.push([valueOrEmpty(it.time), valueOrEmpty(it.machine), valueOrEmpty(it.operator),
                    valueOrEmpty(it.level), valueOrEmpty(it.area), valueOrEmpty(it.mode),
                    valueOrEmpty(it.fill_state), valueOrEmpty(it.dosing_state), valueOrEmpty(it.cr_state),
                    valueOrEmpty(it.message), valueOrEmpty(it.action)])
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
        implicitHeight: 86; radius: 6
        color: cCardBg
        border.color: cBorder; border.width: 1
        GlassHighlight {}
 
        ColumnLayout {
            anchors.centerIn: parent; spacing: 4
            Text {
                text: String(num)
                color: accent; font.pixelSize: 30; font.bold: true; font.family: prodTab.dashboardTextFamily
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: lbl; color: cSection; font.pixelSize: 13; font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
        }
    }
 
    // ── Date input box ──
    component DateBox: Rectangle {
        id: dateBox
        property alias text: dateField.text
        // Emitted when the calendar closes with a day chosen, so the owning
        // section can refresh itself. Typing into the field does not emit —
        // that path still goes through the View button.
        signal dateApplied()

        width: 160; height: 44; radius: 8
        color: "transparent"
        border.color: (dateField.activeFocus || dateCalendar.opened) ? cAccent : cFieldBorder
        border.width: 1
        gradient: Gradient {
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: cFieldStart }
            GradientStop { position: 1.0; color: cFieldEnd }
        }
        SmartTextInput {
            id: dateField
            focusHost: prodTab.focusHost
            anchors.fill: parent; anchors.margins: 8
            color: cFieldText; font.pixelSize: 15; font.family: prodTab.dashboardTextFamily
            clip: true; verticalAlignment: TextInput.AlignVCenter
            inputMask: "99/99/9999; "
            // The calendar owns this field — a numeric pad would let the
            // operator enter 31/02 and would hide the month view behind it.
            useNumpad: false
            inputMethodHints: Qt.ImhDigitsOnly
            Text {
                visible: (dateField.text.trim() === "//" || dateField.text === "  /  /    ") && !dateField.activeFocus
                text: "DD/MM/YYYY"; color: cMuted; font.pixelSize: 14
                anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter
            }
        }

        // Tapping the box opens the month view; typing the date by hand is
        // still possible through the physical keyboard when one is attached.
        MouseArea {
            anchors.fill: parent
            z: 5
            // convertDateFormat returns "" for a half-typed mask, which makes
            // the calendar fall back to today.
            onClicked: dateCalendar.openAt(prodTab.convertDateFormat(dateBox.text))
        }

        CalendarPopup {
            id: dateCalendar
            // Anchored to the field, nudged up when it would fall off-screen.
            parent: dateBox
            x: 0
            y: dateBox.mapToItem(prodTab, 0, dateBox.height).y + height > prodTab.height
               ? -height - 6
               : dateBox.height + 6
            onAccepted: function(isoDate) {
                dateBox.text = prodTab.isoToDisplay(isoDate)
                dateBox.dateApplied()
            }
        }
    }
 
    // ── Action button ──
    component ActionBtn: Rectangle {
        id: actionBtn
        property string label: "Xem"
        signal clicked()
        width: 80; height: 44; radius: 8
        enabled: pendingRequests === 0
        opacity: enabled ? 1.0 : 0.48
        gradient: Gradient {
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: abMA.pressed ? Qt.darker(cActionStart, 1.15) : cActionStart }
            GradientStop { position: 1.0; color: abMA.pressed ? Qt.darker(cActionEnd, 1.15) : cActionEnd }
        }
        border.color: cActionBorder; border.width: 1
        Text { anchors.centerIn: parent; text: actionBtn.label; color: cTitle; font.pixelSize: 15; font.bold: true }
        MotionMouseArea { id: abMA; anchors.fill: parent; onClicked: actionBtn.clicked() }
    }
 
    // ── Data table with headers + rows ──
    component DataTable: Rectangle {
        id: tableRoot
        property var headers: []
        property var colWidths: [] // weights
        property var rows: []
        // Header names whose cells wrap onto several lines and grow the row
        // instead of being cut off with an ellipsis. Empty by default, so every
        // other table keeps its fixed 36 px rows.
        property var wrapColumns: []

        function wrapsColumn(index) {
            var hdr = headers.length > index ? headers[index] : ""
            return wrapColumns.indexOf(hdr) >= 0
        }
 
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
                color: cCardBg
 
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
                                color: cSection
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
                    // A Row positioner reports the tallest child as its implicit
                    // height, so a wrapped cell grows the whole row.
                    implicitHeight: Math.max(36, cellRow.implicitHeight)
                    color: index % 2 === 0 ? "transparent" : cCardBg

                    readonly property int rowIndex: index

                    Row {
                        id: cellRow
                        width: parent.width
                        spacing: 0
                        Repeater {
                            model: rows[rowRect.rowIndex]
                            Item {
                                id: cellItem
                                readonly property bool wraps: tableRoot.wrapsColumn(index)
                                width: tableRoot.getCellWidth(index, cellRow.width)
                                height: Math.max(36, cellText.implicitHeight + 10)

                                // Separator spans the full row, which may be
                                // taller than this cell.
                                Rectangle {
                                    anchors.top: parent.top
                                    anchors.right: parent.right
                                    height: rowRect.height
                                    width: index < headers.length - 1 ? 1 : 0
                                    color: cBorder
                                }

                                Text {
                                    id: cellText
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.margins: 5
                                    anchors.verticalCenter: parent.verticalCenter
                                    wrapMode: cellItem.wraps ? Text.WordWrap : Text.NoWrap
                                    maximumLineCount: cellItem.wraps ? 3 : 1
                                    text: modelData
                                    color: {
                                        var hdr = headers.length > index ? headers[index] : ""
                                        var value = String(modelData).toUpperCase()
                                        if (hdr === "Result" || hdr === "Kết quả") {
                                            return value === "NG" ? cBad : cOk
                                        }
                                        if (hdr === "Level")
                                            return value === "ERROR" ? cBad : (value === "WARN" ? cWarn : cText)
                                        if (hdr === "Event")
                                            return value === "RESOLVED" ? cOk : cWarn
                                        return cText
                                    }
                                    font.pixelSize: 13
                                    font.family: prodTab.dashboardTextFamily
                                    font.bold: {
                                        var hdr = headers.length > index ? headers[index] : ""
                                        return hdr === "Result" || hdr === "Kết quả" || hdr === "Level" || hdr === "Event"
                                    }
                                    horizontalAlignment: cellItem.wraps ? Text.AlignLeft : Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    elide: Text.ElideRight

                                    MouseArea {
                                        id: cellHover
                                        anchors.fill: parent
                                        hoverEnabled: true
                                        acceptedButtons: Qt.NoButton
                                    }
                                    ToolTip.visible: cellHover.containsMouse && cellText.truncated
                                    ToolTip.text: cellText.text
                                    ToolTip.delay: 350
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
