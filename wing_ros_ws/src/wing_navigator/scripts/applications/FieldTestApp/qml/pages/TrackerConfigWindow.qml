import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Window 2.15
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
        id: controlContainer
        width: parent.width * 0.85
        height: parent.height * 0.15
        radius: 10
        color: ThemeManager.m3["surfaceContainerLow"]
        anchors {
            top: formContainer.bottom
            topMargin: 5
            horizontalCenter: parent.horizontalCenter
        }

        CustomTextBtn {
            id: applyBtn
            width: parent.width * 0.5
            height: parent.height * 0.5
            anchors {
                horizontalCenter: parent.horizontalCenter
                verticalCenter: parent.verticalCenter
            }
            btnLabel: qsTr("Apply")
            onClicked: {
                applyTrackerSettingsBtnSignal(parseFloat(distThreshTextField.text), parseInt(initDelayTextField.text, 10), parseInt(hitCountMaxTextField.text, 10));
            }
        }
    }
}
