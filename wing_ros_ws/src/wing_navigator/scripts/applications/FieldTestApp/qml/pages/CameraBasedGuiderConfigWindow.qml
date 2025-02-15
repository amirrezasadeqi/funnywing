import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Window 2.15
import QtQuick.Controls.Material 2.15
import "../controls"
import "../theme" 1.0

Window {
    id: configWindow
    width: 800
    height: 800
    title: "Camera Based Guidance Configuration"
    color: ThemeManager.m3["surface"]
    modality: Qt.NonModal  // Ensures it doesn't block the main window
    visible: false         // Initially hidden
    flags: Qt.Window | Qt.WindowTitleHint | Qt.WindowCloseButtonHint | Qt.CustomizeWindowHint

    // Stores PID values for both controllers
    property var pidControllers: [[], []]
    // Stores sigmoidCoefficients
    property var sigmoidCoefficients: []

    signal cameraBasedGuiderApplyConfigsSignal(var configs)

    Rectangle {
        id: formContainer
        width: parent.width * 0.9
        height: parent.height * 0.8
        radius: 10
        border.color: ThemeManager.m3["outline"]
        color: ThemeManager.m3["surfaceContainerHigh"]
        anchors.top: parent.top
        anchors.topMargin: parent.height * 0.03
        anchors.horizontalCenter: parent.horizontalCenter

        Rectangle {
            id: profileSwitchContainer
            width: 200
            height: 30
            color: "transparent"
            anchors {
                top: parent.top
                topMargin: 20
                left: parent.left
                leftMargin: 10
            }
            Label {
                id: profileTypeLabel
                color: ThemeManager.m3["onSurface"]
                text: profileSwitch.checked ? qsTr("Sigmoid Profile") : qsTr("Constant Profile")
                anchors {
                    verticalCenter: parent.verticalCenter
                    left: parent.left
                    leftMargin: 3
                }
            }
            Switch {
                id: profileSwitch
                Material.accent: ThemeManager.m3["tertiaryContainer"]
                anchors {
                    verticalCenter: parent.verticalCenter
                    right: parent.right
                    rightMargin: 3
                }
            }
        }

        Label {
            id: constThrottleLabel
            text: qsTr("Constant Throttle")
            color: ThemeManager.m3["onSurface"]
            anchors {
                verticalCenter: profileSwitchContainer.verticalCenter
                left: parent.horizontalCenter
            }
        }

        CustomTextField {
            id: constThrottleTextField
            width: 50
            leftPadding: 5
            rightPadding: 5
            bottomPadding: 2
            topPadding: 2
            anchors {
                verticalCenter: constThrottleLabel.verticalCenter
                left: constThrottleLabel.right
                leftMargin: 5
            }
            placeholderText: ""
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            maximumLength: 4
            validator: DoubleValidator {
                bottom: 0.0;
                top: 1.0
            }
            HoverHandler {
                id: constThrottleHoverHandler
            }
            ToolTip.visible: constThrottleHoverHandler.hovered
            ToolTip.text: qsTr("Enter throttle value, [0, 1.0], for the constant profile.")
            ToolTip.delay: 1000
            ToolTip.timeout: 3000
        }

        GroupBox {
            id: xpidGroupBox
            width: parent.width * 0.45
            height: 320
            title: qsTr("X PIDs")
            anchors {
                top: profileSwitchContainer.bottom
                topMargin: 5
                left: parent.left
                leftMargin: 10
            }
            label: Label {
                x: xpidGroupBox.leftPadding
                width: xpidGroupBox.availableWidth
                text: xpidGroupBox.title
                color: ThemeManager.m3["onSurface"]
                elide: Text.ElideRight
            }
            background: Rectangle {
                y: xpidGroupBox.topPadding - xpidGroupBox.bottomPadding
                width: xpidGroupBox.width
                height: xpidGroupBox.height - xpidGroupBox.topPadding + xpidGroupBox.bottomPadding
                color: ThemeManager.m3["surfaceContainerHighest"]
                border.color: ThemeManager.m3["outlineVariant"]
                radius: 5
            }

            CheckDelegate {
                id: xpidDisableCheckBox
                text: qsTr("Disable")
                Material.accent: ThemeManager.m3["tertiaryContainer"]
                contentItem: Text {
                    rightPadding: xpidDisableCheckBox.indicator.width + xpidDisableCheckBox.spacing
                    text: xpidDisableCheckBox.text
                    font: xpidDisableCheckBox.font
                    opacity: enabled ? 1.0 : 0.3
                    color: ThemeManager.m3["onSurface"]
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }
                anchors {
                    top: parent.top
                    topMargin: 5
                    left: parent.left
                    leftMargin: 5
                }
            }

            CheckDelegate {
                id: xpidLockCheckBox
                text: qsTr("Lock")
                Material.accent: ThemeManager.m3["tertiaryContainer"]
                contentItem: Text {
                    rightPadding: xpidLockCheckBox.indicator.width + xpidLockCheckBox.spacing
                    text: xpidLockCheckBox.text
                    font: xpidLockCheckBox.font
                    opacity: enabled ? 1.0 : 0.3
                    color: ThemeManager.m3["onSurface"]
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }
                anchors {
                    top: parent.top
                    topMargin: 5
                    left: xpidDisableCheckBox.right
                    leftMargin: 5
                }
            }

            Rectangle {
                id: xpidContainer
                width: parent.width
                color: "transparent"
                anchors {
                    top: xpidLockCheckBox.bottom
                    topMargin: 5
                    bottom: parent.bottom
                    bottomMargin: 5
                }
                Column {
                    spacing: 10
                    Repeater {
                        model: [
                            {"labelText": "P"},
                            {"labelText": "I"},
                            {"labelText": "D"}
                        ]
                        delegate: PIDConfigRow{
                            width: xpidContainer.width * 0.9
                            labelText: modelData.labelText
                            enabled: !xpidLockCheckBox.checked
                        }
                        onItemAdded: configWindow.pidControllers[0].push(item)
                    }
                }
            }

        }

        GroupBox {
            id: ypidGroupBox
            width: parent.width * 0.45
            height: 320
            title: qsTr("Y PIDs")
            anchors {
                top: profileSwitchContainer.bottom
                topMargin: 5
                right: parent.right
                rightMargin: 10
            }
            label: Label {
                x: ypidGroupBox.leftPadding
                width: ypidGroupBox.availableWidth
                text: ypidGroupBox.title
                color: ThemeManager.m3["onSurface"]
                elide: Text.ElideRight
            }
            background: Rectangle {
                y: ypidGroupBox.topPadding - ypidGroupBox.bottomPadding
                width: ypidGroupBox.width
                height: ypidGroupBox.height - ypidGroupBox.topPadding + ypidGroupBox.bottomPadding
                color: ThemeManager.m3["surfaceContainerHighest"]
                border.color: ThemeManager.m3["outlineVariant"]
                radius: 5
            }
            CheckDelegate {
                id: ypidDisableCheckBox
                text: qsTr("Disable")
                Material.accent: ThemeManager.m3["tertiaryContainer"]
                contentItem: Text {
                    rightPadding: ypidDisableCheckBox.indicator.width + ypidDisableCheckBox.spacing
                    text: ypidDisableCheckBox.text
                    font: ypidDisableCheckBox.font
                    opacity: enabled ? 1.0 : 0.3
                    color: ThemeManager.m3["onSurface"]
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }
                anchors {
                    top: parent.top
                    topMargin: 5
                    left: parent.left
                    leftMargin: 5
                }
            }

            CheckDelegate {
                id: ypidLockCheckBox
                text: qsTr("Lock")
                Material.accent: ThemeManager.m3["tertiaryContainer"]
                contentItem: Text {
                    rightPadding: ypidLockCheckBox.indicator.width + ypidLockCheckBox.spacing
                    text: ypidLockCheckBox.text
                    font: ypidLockCheckBox.font
                    opacity: enabled ? 1.0 : 0.3
                    color: ThemeManager.m3["onSurface"]
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }
                anchors {
                    top: parent.top
                    topMargin: 5
                    left: ypidDisableCheckBox.right
                    leftMargin: 5
                }
            }

            Rectangle {
                id: ypidContainer
                width: parent.width
                color: "transparent"
                anchors {
                    top: ypidLockCheckBox.bottom
                    topMargin: 5
                    bottom: parent.bottom
                    bottomMargin: 5
                }
                Column {
                    spacing: 10
                    Repeater {
                        model: [
                            {"labelText": "P"},
                            {"labelText": "I"},
                            {"labelText": "D"}
                        ]
                        delegate: PIDConfigRow{
                            width: ypidContainer.width * 0.9
                            labelText: modelData.labelText
                            enabled: !ypidLockCheckBox.checked
                        }
                        onItemAdded: configWindow.pidControllers[1].push(item)
                    }
                }
            }
        }

        GroupBox {
            id: sigmoidProfileConstsGroupBox
            title: qsTr("Sigmoid Profile Constants")
            anchors {
                top: xpidGroupBox.bottom
                topMargin: 3
                bottom: parent.bottom
                bottomMargin: 3
                left: parent.left
                leftMargin: 3
                right: parent.right
                rightMargin: 3
            }
            label: Label {
                x: sigmoidProfileConstsGroupBox.leftPadding
                width: sigmoidProfileConstsGroupBox.availableWidth
                text: sigmoidProfileConstsGroupBox.title
                color: ThemeManager.m3["onSurface"]
                elide: Text.ElideRight
            }
            background: Rectangle {
                y: sigmoidProfileConstsGroupBox.topPadding - sigmoidProfileConstsGroupBox.bottomPadding
                width: sigmoidProfileConstsGroupBox.width
                height: sigmoidProfileConstsGroupBox.height - sigmoidProfileConstsGroupBox.topPadding + sigmoidProfileConstsGroupBox.bottomPadding
                color: ThemeManager.m3["surfaceContainerHighest"]
                border.color: ThemeManager.m3["outlineVariant"]
                radius: 5
            }
            Grid {
                id: sigmoidProfileConstsGrid
                anchors.fill: parent
                columns: 2
                rows: 4
                columnSpacing: width * 0.05
                rowSpacing: 3
                Repeater {
                    model: [
                        {"labelName": "a"},
                        {"labelName": "b"},
                        {"labelName": "size_threshold"},
                        {"labelName": "wing_too_below_threshold"},
                        {"labelName": "wing_too_below_throttle"},
                        {"labelName": "wing_too_above_threshold"},
                        {"labelName": "wing_too_above_throttle"},
                        {"labelName": "wing_tg_at_same_level_throttle"}
                    ]
                    delegate: Rectangle {
                        width: sigmoidProfileConstsGroupBox.width * 0.45
                        height: sigmoidProfileConstsGroupBox.height * 0.2
                        color: ThemeManager.m3["surfaceDim"]
                        border.color: ThemeManager.m3["outlineVariant"]
                        radius: 5
                        property string coefficientName: coefficientLabel.text
                        property alias coefficientTextValue: coefficientTextField.text
                        Label {
                            id: coefficientLabel
                            text: modelData.labelName
                            color: ThemeManager.m3["onSurface"]
                            anchors {
                                verticalCenter: parent.verticalCenter
                                left: parent.left
                                leftMargin: 5
                            }
                        }

                        CustomTextField {
                            id: coefficientTextField
                            width: parent.width * 0.28
                            height: parent.height * 0.9
                            leftPadding: 5
                            rightPadding: 5
                            bottomPadding: 2
                            topPadding: 2
                            anchors {
                                right: parent.right
                                rightMargin: 2
                                verticalCenter: parent.verticalCenter
                            }
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            validator: DoubleValidator {}
                        }
                    }
                    onItemAdded: configWindow.sigmoidCoefficients.push(item)
                }
            }
        }
    }

    Rectangle {
        id: controlContainer
        width: formContainer.width
        height: configWindow.height * 0.15
        color: ThemeManager.m3["surfaceContainer"]
        border.color: ThemeManager.m3["outlineVariant"]
        radius: 5
        anchors.top: formContainer.bottom
        anchors.topMargin: 5
        anchors.horizontalCenter: parent.horizontalCenter

        Row {
            id: controlContainerRow
            spacing: 5
            anchors {
                verticalCenter: parent.verticalCenter
                right: parent.right
                rightMargin: 10
            }
            CustomTextBtn {
                id: applyBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Apply")
                width: controlContainer.width * 0.2
                onClicked: {
                    let profileType = (profileSwitch.checked) ? 1 : 0;
                    let constThrottle = parseFloat(constThrottleTextField.text);
                    let xpids = [
                            configWindow.pidControllers[0][0].value,
                            configWindow.pidControllers[0][1].value,
                            configWindow.pidControllers[0][2].value,
                        ];
                    let ypids = [
                            configWindow.pidControllers[1][0].value,
                            configWindow.pidControllers[1][1].value,
                            configWindow.pidControllers[1][2].value,
                        ];
                    let sigmoidCoeff = [
                            parseFloat(configWindow.sigmoidCoefficients[0].coefficientTextValue),
                            parseFloat(configWindow.sigmoidCoefficients[1].coefficientTextValue),
                            parseFloat(configWindow.sigmoidCoefficients[2].coefficientTextValue),
                            parseFloat(configWindow.sigmoidCoefficients[3].coefficientTextValue),
                            parseFloat(configWindow.sigmoidCoefficients[4].coefficientTextValue),
                            parseFloat(configWindow.sigmoidCoefficients[5].coefficientTextValue),
                            parseFloat(configWindow.sigmoidCoefficients[6].coefficientTextValue),
                            parseFloat(configWindow.sigmoidCoefficients[7].coefficientTextValue)
                        ];

                    var configs = {
                        "profile_type": profileType,
                        "const_throttle": constThrottle,
                        "x_pids": (!xpidDisableCheckBox.checked) ? xpids : [0, 0, 0],
                        "y_pids": (!ypidDisableCheckBox.checked) ? ypids : [0, 0, 0],
                        "a": sigmoidCoeff[0],
                        "b": sigmoidCoeff[1],
                        "size_threshold": sigmoidCoeff[2],
                        "wing_too_below_threshold": sigmoidCoeff[3],
                        "wing_too_below_throttle": sigmoidCoeff[4],
                        "wing_too_above_threshold": sigmoidCoeff[5],
                        "wing_too_above_throttle": sigmoidCoeff[6],
                        "wing_tg_at_same_level_throttle": sigmoidCoeff[7]
                    };

                    configWindow.cameraBasedGuiderApplyConfigsSignal(configs);
                }
            }
            CustomTextBtn {
                id: saveBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Save")
                width: controlContainer.width * 0.2
            }
            CustomTextBtn {
                id: loadBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Load")
                width: controlContainer.width * 0.2
            }
        }
    }
}
