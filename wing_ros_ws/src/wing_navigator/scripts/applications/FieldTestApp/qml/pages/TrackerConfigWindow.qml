import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Window 2.15
import Qt.labs.platform 1.1 
import QtQuick.Layouts 1.3
import Qt.labs.settings 1.0
import "../controls"
import "../theme" 1.0

Window {
    id: trackerConfigWindow
    width: 500
    height: 600
    title: "Tracker Configuration"
    color: ThemeManager.m3["surface"]
    modality: Qt.NonModal
    visible: false
    flags: Qt.Window | Qt.WindowTitleHint | Qt.WindowCloseButtonHint | Qt.CustomizeWindowHint

    signal applyTrackerSettingsBtnSignal(real distThresh, int initDelay, int hitCountMax)

    Rectangle {
        id: formContainer
        width: parent.width * 0.85
        height: parent.height * 0.7
        radius: 10
        border.color: ThemeManager.m3["outline"]
        color: ThemeManager.m3["surfaceContainerHigh"]
        anchors {
            horizontalCenter: parent.horizontalCenter
            top: parent.top
            topMargin: parent.height * 0.08
        }

        Label {
            id: distThreshLabel
            text: qsTr("Distance Threshold")
            color: ThemeManager.m3["onSurface"]
            anchors {
                left: parent.left
                leftMargin: parent.width * 0.08
                top: parent.top
                topMargin: parent.height * 0.1
            }
        }

        CustomTextField {
            id: distThreshTextField
            width: parent.width * 0.45
            leftPadding: 5
            rightPadding: 5
            bottomPadding: 2
            topPadding: 2
            anchors {
                verticalCenter: distThreshLabel.verticalCenter
                right: parent.right
                rightMargin: parent.width * 0.08
            }
            placeholderText: qsTr("")
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            validator: DoubleValidator {
                bottom: 0
            }
            ToolTip.visible: hovered
            ToolTip.text: qsTr("Higher value is better for faster objects.")
            ToolTip.delay: 1000
            ToolTip.timeout: 3000
        }

        Label {
            id: initDelayLabel
            text: qsTr("Initialization Delay")
            color: ThemeManager.m3["onSurface"]
            anchors {
                left: distThreshLabel.left
                top: distThreshLabel.bottom
                topMargin: 50
            }
        }

        CustomTextField {
            id: initDelayTextField
            width: parent.width * 0.45
            leftPadding: 5
            rightPadding: 5
            bottomPadding: 2
            topPadding: 2
            anchors {
                verticalCenter: initDelayLabel.verticalCenter
                right: distThreshTextField.right
            }
            placeholderText: qsTr("")
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            validator: IntValidator {
                bottom: 0
            }
            ToolTip.visible: hovered
            ToolTip.text: qsTr("Some sort of threshold for minimal number of consequent detections leading to track initialization.")
            ToolTip.delay: 1000
            ToolTip.timeout: 3000
        }

        Label {
            id: hitCountMaxLabel
            text: qsTr("hit_counter_max")
            color: ThemeManager.m3["onSurface"]
            anchors {
                left: distThreshLabel.left
                top: initDelayLabel.bottom
                topMargin: 50
            }
        }

        CustomTextField {
            id: hitCountMaxTextField
            width: parent.width * 0.45
            leftPadding: 5
            rightPadding: 5
            bottomPadding: 2
            topPadding: 2
            anchors {
                verticalCenter: hitCountMaxLabel.verticalCenter
                right: distThreshTextField.right
            }
            placeholderText: qsTr("")
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            validator: IntValidator {
                bottom: 0
            }
            ToolTip.visible: hovered
            ToolTip.text: qsTr("Some sort of track life time. Bigger this value leads to more life time of the track after last detection matching.")
            ToolTip.delay: 1000
            ToolTip.timeout: 3000
        }
    }

    Rectangle {
        id: buttonBar
        width: formContainer.width
        height: 60
        color: ThemeManager.m3["surfaceContainer"]
        radius: 5
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        anchors.horizontalCenter: parent.horizontalCenter

        RowLayout {
            anchors.centerIn: parent
            spacing: 15

            CustomTextBtn {
                id: applyBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Apply")
                Layout.preferredWidth: 120
                onClicked: {
                    trackerConfigWindow.applyTrackerSettingsBtnSignal(
                        parseFloat(distThreshTextField.text),
                        parseInt(initDelayTextField.text, 10),
                        parseInt(hitCountMaxTextField.text, 10)
                    );
                }
            }

            CustomTextBtn {
                id: saveBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Save")
                Layout.preferredWidth: 120
                onClicked: { saveDialog.open() }
            }

            CustomTextBtn {
                id: loadBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Load")
                Layout.preferredWidth: 120
                onClicked: { loadDialog.open() }
            }
        }
    }

    Settings {
        id: trackerSettings
        fileName: StandardPaths.writableLocation(StandardPaths.DocumentsLocation) + "/tracker_config.ini"
        property real distanceThreshold: 0
        property int initDelay: 0
        property int hitCountMax: 0
    }

    FileDialog {
        id: saveDialog
        title: "Save Tracker Config"
        fileMode: FileDialog.SaveFile
        nameFilters: ["Config files (*.ini)", "All files (*)"]

        onAccepted: {
            var path = saveDialog.file.toString().replace("file://", "")
            trackerSettings.fileName = path
            trackerSettings.distanceThreshold = parseFloat(distThreshTextField.text)
            trackerSettings.initDelay = parseInt(initDelayTextField.text, 0)
            trackerSettings.hitCountMax = parseInt(hitCountMaxTextField.text, 0)
            trackerSettings.sync()
            console.log("Saved tracker config to:", path)
        }
    }

    FileDialog {
        id: loadDialog
        title: "Load Tracker Config"
        fileMode: FileDialog.OpenFile
        nameFilters: ["Config files (*.ini)", "All files (*)"]

        onAccepted: {
            var path = loadDialog.file.toString().replace("file://", "")
            trackerSettings.fileName = path
            trackerSettings.sync()
            distThreshTextField.text = trackerSettings.distanceThreshold.toString()
            initDelayTextField.text = trackerSettings.initDelay.toString()
            hitCountMaxTextField.text = trackerSettings.hitCountMax.toString()
            console.log("Loaded tracker config from:", path)
        }
    }
}
