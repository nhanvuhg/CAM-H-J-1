import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: settingsWindow
    width: 720
    height: 480
    title: "Settings"
    flags: Qt.Window
    color: "#06101d"

    property var availableTopics: []
    property var selectedTopics: []
    property bool loadingTopics: true

    Shortcut {
        sequence: "Escape"
        onActivated: settingsWindow.close()
    }

    Component.onCompleted: {
        Qt.callLater(() => {
            x = (mainWindow.width - width) / 2 + mainWindow.x
            y = (mainWindow.height - height) / 2 + mainWindow.y
        });
        // Initialize selectedTopics — ensure no undefined entries
        let list = camNode.cameraList;
        let topics = [];
        for (let i = 0; i < list.length; ++i)
            topics.push(list[i].topic || "");
        selectedTopics = topics;
        // Trigger async discovery — does NOT block UI
        camNode.fetchAvailableTopicsAsync();
    }

    // Receive result from background thread
    Connections {
        target: camNode
        function onAvailableTopicsChanged(topics) {
            availableTopics = topics
            loadingTopics = false
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.leftMargin: 30
            Layout.rightMargin: 30
            Layout.topMargin: 20
            color: "transparent"
            border.color: "#163a52"
            border.width: 1
            radius: 6

            Text {
                anchors.centerIn: parent
                visible: settingsWindow.loadingTopics
                text: "⏳ Scanning ROS2 topics..."
                color: "#67d0ff"; font.pixelSize: 16; font.bold: true
            }
            GridLayout {
                columns: 1
                anchors.fill: parent
                anchors.margins: 10
                columnSpacing: 15
                visible: !settingsWindow.loadingTopics

                Repeater {
                    model: camNode.cameraList

                    delegate: ColumnLayout {
                        property int index: model.index
                        spacing: 4

                        Text {
                            text: modelData.name
                            color: "#67d0ff"
                            font.pixelSize: 16
                        }

                        ComboBox {
                            id: topicCombo
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40

                            model: availableTopics
                            currentIndex: availableTopics.indexOf(modelData.topic)

                            onActivated: (topicIndex) => {
                                let selected = availableTopics[topicIndex]
                                if (!selected || selected === "") return;
                                // Copy array to trigger QML binding update
                                let tmp = selectedTopics.slice();
                                tmp[index] = selected;
                                selectedTopics = tmp;
                                modelData.topic = selected
                            }

                            background: Rectangle {
                                color: "transparent"
                                border.color: "#163a52"
                                border.width: 1
                                radius: 6
                            }

                            contentItem: Text {
                                text: topicCombo.displayText
                                font.pixelSize: 14
                                color: "#67d0ff"
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignLeft
                                elide: Text.ElideRight
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 10
                            }
                        }
                    }
                }
            }
        }

        MotionButton {
            text: "Save"
            Layout.alignment: Qt.AlignRight
            Layout.topMargin: 0
            Layout.bottomMargin: 30
            Layout.rightMargin: 30
            Layout.preferredHeight: 40
            Layout.preferredWidth: 120

            contentItem: Text {
                text: "Save"
                color: "#67d0ff"
                font.pixelSize: 14
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                anchors.fill: parent
            }

            background: Rectangle {
                radius: 6
                color: "transparent"
                border.color: "#163a52"
                border.width: 2
            }

            onClicked: {
                for (let i = 0; i < selectedTopics.length; ++i) {
                    let t = selectedTopics[i];
                    if (!t || t === "") {
                        console.warn("Skipping camera", i, "— no topic selected")
                        continue;
                    }
                    camNode.updateCameraTopic(i, t)
                    console.log("Updated camera", i, "to topic:", t)
                }
                settingsWindow.close();
            }
        }
    }
}
