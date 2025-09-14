import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import QtGraphicalEffects 1.15
import "../controls"
import "../theme" 1.0

Item {
    id: configView
    implicitWidth: 1171
    implicitHeight: 250

    property color labelColor: ThemeManager.m3["onSurface"]
    property color labelBgColor: ThemeManager.m3["surfaceDim"]
    property color borderColor: ThemeManager.m3["outlineVariant"]
    property color actionRectBg: ThemeManager.m3["surfaceContainerLow"]
    property color defaultColorBg: ThemeManager.m3["secondaryContainer"]

    signal testScenarioBtnSignal(int scenarioIdx, bool active)
    signal setSettingsBtnSignal(real waypointRadius, bool local, bool wingAsVirtualCenter)
    signal setApParamBtnSignal(string paramName, real paramValue)

    Rectangle {
        id: bg
        color: ThemeManager.m3["surfaceBright"]
        anchors.fill: parent

        ScrollView {
            id: scrollView
            anchors.fill: parent

        ColumnLayout {
            id: mainLayout
            anchors.fill: parent
            anchors.margins: 15
            spacing: 10 

            Rectangle {
                id: testActionContainer
                Layout.fillWidth: true
                Layout.preferredHeight: 120
                color: actionRectBg
                border.color: borderColor
                border.width: 1
                radius: 3

                Rectangle {
                    id: testActionLabelContaienr
                    width: parent.width
                    height: 40
                    color: labelBgColor
                    border.color: borderColor
                    border.width: 1
                    anchors {
                        left: parent.left
                        top: parent.top
                    }
        
                    Label {
                        id: testActionLabel
                        text: "GPS Guidance"
                        color: labelColor
                        font.pointSize: 20
                        anchors.centerIn: parent
                    }   
                }

                ColumnLayout {
                    anchors.top: testActionLabelContaienr.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom
                    anchors.margins: 10
                    spacing: 40

                    RowLayout {
                        spacing: 150
                        Layout.fillWidth: true
            
                        TextField {
                            id: waypointRadiusCustomTextfield
                            Layout.preferredWidth: 400
                            Layout.preferredHeight: 50
                            Layout.topMargin: -15
                            leftPadding: 10
                            topPadding: 15
                            Material.accent: ThemeManager.m3["tertiary"]
                            placeholderText: "Enter Virtual Target Offset"
                            color: ThemeManager.m3["onSurface"]
                            placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 2)
                            selectByMouse: true
                            selectedTextColor: ThemeManager.m3["onSurface"]
                            selectionColor: ThemeManager.m3["inversePrimary"]
                            ToolTip.visible: hovered
                            ToolTip.text: qsTr("Enter a value 5 meters above the wing loiter radius.")
                            ToolTip.delay: 1000
                            ToolTip.timeout: 3000
                            font.pointSize: 15
                            background: Rectangle {
                                implicitWidth: 200
                                implicitHeight: 40
                                radius: 10
                                color: ThemeManager.m3["surfaceContainerLowest"]
                                border {
                                    color: borderColor
                                    width: 1
                                }
                            }
                        }

                        Item { Layout.preferredWidth: 50 }

            
                        ColumnLayout {
                            spacing: 5

                            Label {
                                id: localityExecutionLabel
                                color: labelColor
                                text: qsTr("Local Execution")
                                font.pointSize: 18
                                Layout.alignment: Text.AlignHCenter
                                horizontalAlignment: Text.AlignHCenter
                                Layout.preferredWidth: 28
                            }

                            Switch {
                                id: localityExecutionSwitch
                                Layout.alignment: Qt.AlignHCenter
                                property int switchWidth: 80
                                property int switchHeight: 30

                                indicator: Rectangle {
                                    implicitWidth: localityExecutionSwitch.switchWidth
                                    implicitHeight: localityExecutionSwitch.switchHeight
                                    radius: height / 2
                                    color: localityExecutionSwitch.checked
                                    ? ThemeManager.m3["tertiaryContainer"]
                                    : "#ccc"

                                    Rectangle {
                                        width: parent.height - 6
                                        height: parent.height - 6
                                        radius: width / 2
                                        y: 3
                                        x: localityExecutionSwitch.checked
                                        ? parent.width - width - 3
                                        : 3
                                        color: "white"
                                    }
                                }

                                onToggled: {
                                    if (!checked) {
                                        localityExecutionLabel.text = qsTr("Local Execution");
                                    } else {
                                        localityExecutionLabel.text = qsTr("Remote Execution");
                                     }
                                }
                            }
                        }

                        Item { Layout.preferredWidth: 50 }

                        ColumnLayout {
                            spacing: 5
                
                            Label {
                                id: virtualCenterLabel
                                color: labelColor
                                text: qsTr("Wing as Virtual Center")
                                font.pointSize: 18
                                Layout.alignment: Text.AlignHCenter
                                horizontalAlignment: Text.AlignHCenter
                                Layout.preferredWidth: 28
                            }
                
                            Switch {
                                id: virtualCenterSwitch
                                property int switchWidth: 80
                                property int switchHeight: 30
                                Layout.alignment: Qt.AlignHCenter

                                indicator: Rectangle {
                                    implicitWidth: virtualCenterSwitch.switchWidth
                                    implicitHeight: virtualCenterSwitch.switchHeight
                                    radius: height / 2
                                    color: virtualCenterSwitch.checked
                                    ? ThemeManager.m3["tertiaryContainer"]
                                    : "#ccc"

                                    Rectangle {
                                        width: parent.height - 6
                                        height: parent.height - 6
                                        radius: width / 2
                                        y: 3
                                        x: virtualCenterSwitch.checked
                                        ? parent.width - width - 3
                                        : 3
                                    color: "white"
                                    }
                                }
                                onToggled: {
                                    if (!virtualCenterSwitch.checked) {
                                        virtualCenterLabel.text = qsTr("Wing as Virtual Center");
                                    } else {
                                        virtualCenterLabel.text = qsTr("Target as Virtual Center");
                                     }
                                }
                            }
                        }

                        Item { Layout.preferredWidth: 530 }

                        CustomTextBtn {
                            id: setSimpleTrackerSettingsBtn
                            Layout.preferredWidth: 240
                            Layout.preferredHeight: 60
                            Layout.alignment: Qt.AlignVCenter 
                            Layout.bottomMargin: 15
                            defaultColor: defaultColorBg
                            font.pointSize: 10
                            btnLabel: "Set Settings"
                            onClicked: {
                                let radius = (waypointRadiusCustomTextfield.text.length === 0) ?
                                120.0 : parseFloat(waypointRadiusCustomTextfield.text);
                                actionView.setSettingsBtnSignal(radius, !localityExecutionSwitch.checked, !virtualCenterSwitch.checked);
                            }
                        }
                    }
                }
            }

            Rectangle {
                id: arduplaneContainer
                Layout.fillWidth: true
                Layout.preferredHeight: 300
                color: actionRectBg
                border.color: borderColor
                border.width: 1
                radius: 3

                Rectangle {
                    id: arduplaneLabelContainer
                    width: parent.width
                    height: 40
                    color: labelBgColor
                    border.color: borderColor
                    border.width: 1
                    anchors {
                        left: parent.left
                        top: parent.top
                    }

                    Label {
                        id: arduplaneLabel
                        text: "ArduPlane Configurations"
                        color: labelColor
                        font.pointSize: 20
                        anchors.centerIn: parent
                    }
                }               

                RowLayout {
                    anchors.top: arduplaneLabelContainer.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom
                    anchors.margins: 15
                    spacing: 20

                    Rectangle {
                        id: flightRadiusSection
                        Layout.fillWidth: true
                        Layout.preferredHeight: parent.height
                        color: ThemeManager.m3["surfaceContainerLowest"]
                        border { color: borderColor; width: 1 }
                        radius: 5

                        Rectangle {
                            id: flightRadiusHeader
                            width: parent.width
                            height: 40
                            color: labelBgColor
                            border.color: borderColor
                            border.width: 1
                            anchors.left: parent.left
                            anchors.top: parent.top

                            Label {
                                text: qsTr("Flight Radius Config")
                                color: labelColor
                                anchors.centerIn: parent
                                font.pointSize: 16
                            }
                        }

                        ColumnLayout {
                            anchors.top: flightRadiusHeader.bottom
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.bottom: parent.bottom
                            anchors.margins: 10
                            spacing: 10

                            Loader {
                                id: wpLoiterRadSubLoader
                                sourceComponent: submitControlComp
                                onLoaded: {
                                    item.paramID = qsTr("WP_LOITER_RAD")
                                    item.compLabelText = qsTr("Waypoint Loiter Radius")
                                    item.compTextFieldPlaceHolderText = qsTr("Default: 120")
                                    item.compTextFieldTooltipText = qsTr("The radius that wing tries to hold when turning around waypoint.")
                                }
                            }
                            Loader {
                                id: wpRadSubLoader
                                sourceComponent: submitControlComp
                                onLoaded: {
                                    item.paramID = qsTr("WP_RADIUS")
                                    item.compLabelText = qsTr("Waypoint Radius")
                                    item.compTextFieldPlaceHolderText = qsTr("Default: 120")
                                    item.compTextFieldTooltipText = qsTr("The radius within that the waypoint is considered to be reached.")
                                }
                            }
                        }
                    }
                        

                    Rectangle {
                        id: controllerSection
                        Layout.fillWidth: true
                        Layout.preferredHeight: parent.height
                        color: ThemeManager.m3["surfaceContainerLowest"]
                        border { color: borderColor; width: 1 }
                        radius: 5

                        Rectangle {
                            id: controllerHeader
                            width: parent.width
                            height: 40
                            color: labelBgColor
                            border.color: borderColor
                            border.width: 1
                            anchors.left: parent.left
                            anchors.top: parent.top

                            Label {
                                text: qsTr("Controller Params")
                                color: labelColor
                                anchors.centerIn: parent
                                font.pointSize: 16
                            }
                        }

                        ColumnLayout {
                            anchors.top: controllerHeader.bottom
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.bottom: parent.bottom
                            anchors.margins: 10
                            spacing: 10

                            Loader {
                                id: navL1PeriodLoader
                                sourceComponent: submitControlComp
                                onLoaded: {
                                    item.paramID = qsTr("NAVL1_PERIOD")
                                    item.compLabelText = qsTr("NAVL1_PERIOD")
                                    item.compTextFieldPlaceHolderText = qsTr("Default: 17")
                                    item.compTextFieldTooltipText = qsTr("Time constant of the L1 controller. Smaller value, more aggressive turns. Too small value leads to stall.")
                                }
                            }
                            Loader {
                                id: limRollCdLoader
                                sourceComponent: submitControlComp
                                onLoaded: {
                                    item.paramID = qsTr("LIM_ROLL_CD")
                                    item.compLabelText = qsTr("LIM_ROLL_CD")
                                    item.compTextFieldPlaceHolderText = qsTr("Default: 20")
                                    item.compTextFieldTooltipText = qsTr("Maximum bank angle commanded in modes with stabilized limits. Increase this value for sharper turns.")
                                }
                            }
                        }
                    }

                    Rectangle {
                        id: arduplaneParamsSection
                        Layout.fillWidth: true
                        Layout.preferredHeight: parent.height
                        color: ThemeManager.m3["surfaceContainerLowest"]
                        border { color: borderColor; width: 1 }
                        radius: 5

                        Rectangle {
                            id: ardupalneParamsHeader
                            width: parent.width
                            height: 40
                            color: labelBgColor
                            border.color: borderColor
                            border.width: 1
                            anchors.left: parent.left
                            anchors.top: parent.top

                            Label {
                                text: qsTr("ArduPlane Params Config")
                                color: labelColor
                                anchors.centerIn: parent
                                font.pointSize: 16
                            }
                        }

                        ColumnLayout {
                            anchors.top: ardupalneParamsHeader.bottom
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.bottom: parent.bottom
                            anchors.margins: 10
                            spacing: 15
                
                            ColumnLayout {
                                Label {
                                    color: labelColor
                                    text: qsTr("ArduPlane Parameter ID")
                                }
                                TextField {
                                    id: apParamIDTextField
                                    leftPadding: 10
                                    topPadding: 15
                                    Material.accent: ThemeManager.m3["tertiary"]
                                    selectByMouse: true
                                    color: ThemeManager.m3["onSurface"]
                                    selectedTextColor: ThemeManager.m3["onSurface"]
                                    selectionColor: ThemeManager.m3["inversePrimary"]
                                    placeholderTextColor: ThemeManager.m3["onSurface"]
                                    placeholderText: qsTr("e.g. LIM_PITCH_MIN")
                                    ToolTip.visible: hovered
                                    ToolTip.text: qsTr("ID or name of the parameters defined in the installed version of arduplane's parameter list.")
                                    ToolTip.delay: 1000
                                    ToolTip.timeout: 3000
                                    background: Rectangle {
                                        id: apParamIDTextFieldBg
                                        implicitWidth: 250
                                        implicitHeight: 30
                                        radius: 10
                                        color: ThemeManager.m3["surfaceContainerLowest"]
                                        border { color: borderColor; width: 1 }
                                    }
                                }
                            }

                            ColumnLayout {
                                Label {
                                    color: labelColor
                                    text: qsTr("ArduPlane Parameter Value")
                                }
                                RowLayout {
                                    TextField {
                                        id: apParamValueTextField
                                        Layout.fillWidth: true
                                        leftPadding: 10
                                        topPadding: 15
                                        Material.accent: ThemeManager.m3["tertiary"]
                                        selectByMouse: true
                                        color: ThemeManager.m3["onSurface"]
                                        selectedTextColor: ThemeManager.m3["onSurface"]
                                        selectionColor: ThemeManager.m3["inversePrimary"]
                                        placeholderTextColor: ThemeManager.m3["onSurface"]
                                        placeholderText: qsTr("Enter Value ...")
                                        ToolTip.visible: hovered
                                        ToolTip.text: qsTr("Value of the specified parameter in parameter ID text field.")
                                        ToolTip.delay: 1000
                                        ToolTip.timeout: 3000
                                        background: Rectangle {
                                            id: apParamValueTextFieldBg
                                            implicitWidth: 250
                                            implicitHeight: 30
                                            radius: 10
                                            color: ThemeManager.m3["surfaceContainerLowest"]
                                            border { color: borderColor; width: 1 }
                                        }
                                    }
                                    CustomTextBtn {
                                        id: allApParamSetBtn
                                        width: 20
                                        height: 20
                                        defaultColor: defaultColorBg
                                        contentItem: Image {
                                            id: allApParamSetBtnImage
                                            anchors.fill: parent
                                            source: "../../images/svg_images/submitIcon.svg"
                                            fillMode: Image.PreserveAspectFit
                                            ColorOverlay {
                                                anchors.fill: parent
                                                source: allApParamSetBtnImage
                                                color: ThemeManager.m3["onSecondaryContainer"]
                                            }
                                        }
                                        onClicked: {
                                            actionView.setApParamBtnSignal(apParamIDTextField.text, parseFloat(apParamValueTextField.text))
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Component {
                id: submitControlComp
                Rectangle {
                    id: bg
                    implicitWidth: compTextField.width + compSetBtn.width + 5
                    implicitHeight: compLabel.height + compTextField.height + 5
                    color: "transparent"
                    property string compLabelText: qsTr("LIM_ROLL_CD")
                    property string paramID: qsTr("LIM_ROLL_CD")
                    property string compTextFieldPlaceHolderText: qsTr("Default: 20")
                    property string compTextFieldTooltipText: qsTr("Some Explanations about this input.")
                    property string compSetBtnImageSrc: "../../images/svg_images/submitIcon.svg"

                    ColumnLayout {
                        Label {
                            id: compLabel
                            color: labelColor
                            text: bg.compLabelText
                        }
                        RowLayout {
                            spacing: 5
                            TextField {
                                id: compTextField
                                Layout.fillWidth: true
                                leftPadding: 10
                                topPadding: 15
                                Material.accent: ThemeManager.m3["tertiary"]
                                selectByMouse: true
                                color: ThemeManager.m3["onSurface"]
                                selectedTextColor: ThemeManager.m3["onSurface"]
                                selectionColor: ThemeManager.m3["inversePrimary"]
                                placeholderText: bg.compTextFieldPlaceHolderText
                                placeholderTextColor: ThemeManager.m3["onSurface"]
                                ToolTip.visible: hovered
                                ToolTip.text: bg.compTextFieldTooltipText
                                ToolTip.delay: 1000
                                ToolTip.timeout: 3000
                                background: Rectangle {
                                    id: compTextFieldBg
                                    implicitWidth: 250
                                    implicitHeight: 30
                                    radius: 10
                                    color: ThemeManager.m3["surfaceContainerLowest"]
                                    border { color: borderColor; width: 1 }
                                }
                            }
                            CustomTextBtn {
                                id: compSetBtn
                                width: 50
                                height: 50
                                defaultColor: defaultColorBg
                                contentItem: Image {
                                    id: compSetBtnImage
                                    anchors.fill: parent
                                    source: bg.compSetBtnImageSrc
                                    fillMode: Image.PreserveAspectFit
                                    ColorOverlay {
                                        anchors.fill: parent
                                        source: compSetBtnImage
                                        color: ThemeManager.m3["onSecondaryContainer"]
                                    }
                                }
                                onClicked: {
                                    actionView.setApParamBtnSignal(bg.paramID, parseFloat(compTextField.text))
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
}
