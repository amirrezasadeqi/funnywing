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
        clip: true

        ScrollView {
            id: scrollView
            anchors.fill: parent

            ColumnLayout {
                id: mainLayout
                width: scrollView.availableWidth
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

                    Item {
                        id: gpsFitWrapper 
                        anchors.top: testActionLabelContaienr.bottom
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        anchors.margins: 10

                        Item {
                            id: gpsContent
                            width: Math.max(1500, gpsFitWrapper.width)
                            height: parent.height
                            anchors.left: parent.left
                            anchors.right: undefined

                            transformOrigin: Item.TopLeft   

                            readonly property real scaleFactor: Math.min(
                                gpsFitWrapper.width / 1500,
                                1.0
                            )

                            scale: scaleFactor

                            ColumnLayout {
                                anchors.fill: parent
                                spacing: 20

                                RowLayout {
                                    Layout.fillWidth: true
                                    spacing: 20

                                    TextField {
                                        id: waypointRadiusCustomTextfield
                                        Layout.preferredWidth: 380
                                        Layout.maximumWidth: 380
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
                                            implicitWidth: 100
                                            implicitHeight: 40
                                            radius: 10
                                            color: ThemeManager.m3["surfaceContainerLowest"]
                                            border {
                                                color: borderColor
                                                width: 1
                                            }
                                        }
                                    }

                                    Item {
                                        Layout.fillWidth: true
                                    }

                                    RowLayout {
                                        spacing: 200
                                        Layout.alignment: Qt.AlignHCenter

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
                                    }

                                    Item { 
                                        Layout.fillWidth: true
                                     }

                                    CustomTextBtn {
                                        id: setSimpleTrackerSettingsBtn
                                        Layout.preferredWidth: 240
                                        Layout.preferredHeight: 50
                                        Layout.alignment: Qt.AlignRight | Qt.AlignVCenter 
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

                    Item {
                        id: arduplaneFitWrapper
                        anchors.top: arduplaneLabelContainer.bottom
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        anchors.margins: 15

                        Item {
                            id: arduplaneContent
                            width: 1500
                            height: parent.height
                            anchors.left: parent.left
                            anchors.right: parent.right

                            readonly property real scaleFactor: Math.min(
                                arduplaneFitWrapper.width / width,
                                1.0
                            )

                            scale: scaleFactor
                            
                            RowLayout {
                                anchors.fill: parent
                                spacing: 32

                                Rectangle {
                                    id: flightRadiusSection
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 460
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
                                        spacing: 15
                                        
                                        ColumnLayout {
                                            Label {
                                                text: qsTr("Waypoint Loiter Radius")
                                                color: labelColor
                                            }
                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 10
                                                TextField {
                                                    id: wpLoiterRadField
                                                    Layout.fillWidth: true
                                                    leftPadding: 10
                                                    topPadding: 15
                                                    Material.accent: ThemeManager.m3["tertiary"]
                                                    selectByMouse: true
                                                    color: ThemeManager.m3["onSurface"]
                                                    selectedTextColor: ThemeManager.m3["onSurface"]
                                                    selectionColor: ThemeManager.m3["inversePrimary"]
                                                    placeholderText: qsTr("Default: 120")
                                                    placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 1.8)
                                                    ToolTip.visible: hovered
                                                    ToolTip.text: qsTr("The radius that wing tries to hold when turning around waypoint.")
                                                    ToolTip.delay: 1000
                                                    ToolTip.timeout: 3000
                                                    background: Rectangle {
                                                        implicitWidth: 250
                                                        implicitHeight: 30
                                                        radius: 10
                                                        color: ThemeManager.m3["surfaceContainerLowest"]
                                                        border { color: borderColor; width: 1 }
                                                    }
                                                }
                                                CustomTextBtn {
                                                    id: wpLoiterRadSetBtn
                                                    Layout.preferredHeight: 60
                                                    Layout.preferredWidth: 120
                                                    defaultColor: defaultColorBg
                                                    contentItem: Image {
                                                        id: wpLoiterRadSetBtnImage
                                                        anchors.fill: parent
                                                        source: "../../images/svg_images/submitIcon.svg"
                                                        fillMode: Image.PreserveAspectFit
                                                        ColorOverlay {
                                                            anchors.fill: parent
                                                            source: wpLoiterRadSetBtnImage
                                                            color: ThemeManager.m3["onSecondaryContainer"]
                                                        }
                                                    }
                                                    onClicked: {
                                                        actionView.setApParamBtnSignal(wpLoiterRadField.text, parseFloat(wpLoiterRadField.text))
                                                    }
                                                }
                                            }
                                        }
                                        ColumnLayout {
                                            Label {
                                                text: qsTr("Waypoint Radius")
                                                color: labelColor
                                            }
                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 10
                                                TextField {
                                                    id: wpRadiusField
                                                    Layout.fillWidth: true
                                                    leftPadding: 10
                                                    topPadding: 15
                                                    Material.accent: ThemeManager.m3["tertiary"]
                                                    selectByMouse: true
                                                    color: ThemeManager.m3["onSurface"]
                                                    selectedTextColor: ThemeManager.m3["onSurface"]
                                                    selectionColor: ThemeManager.m3["inversePrimary"]
                                                    placeholderText: qsTr("Default: 120")
                                                    placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 1.8)
                                                    ToolTip.visible: hovered
                                                    ToolTip.text: qsTr("The radius within that the waypoint is considered to be reched.")
                                                    ToolTip.delay: 1000
                                                    ToolTip.timeout: 3000

                                                    background: Rectangle {
                                                        implicitWidth: 250
                                                        implicitHeight: 30
                                                        radius: 10
                                                        color: ThemeManager.m3["surfaceContainerLowest"]
                                                        border { color: borderColor; width: 1}
                                                    }
                                                }
                                                CustomTextBtn {
                                                    id: wpRadiusSetBtn
                                                    Layout.preferredHeight: 60
                                                    Layout.preferredWidth: 120
                                                    defaultColor: defaultColorBg
                                                    contentItem: Image {
                                                        id: wpRadiusSetBtnImage
                                                        anchors.fill: parent
                                                        source: "../../images/svg_images/submitIcon.svg"
                                                        fillMode: Image.PreserveAspectFit
                                                        ColorOverlay {
                                                            anchors.fill: parent
                                                            source: wpRadiusSetBtnImage
                                                            color: ThemeManager.m3["onSecondaryContainer"]
                                                        }
                                                    }
                                                    onClicked: {
                                                        actionView.setApParamBtnSignal(wpRadiusField.text, parseFloat(wpRadiusField.text))
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                 Rectangle {
                                    id: controllerSection
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 460
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
                                        spacing: 15

                                        ColumnLayout {
                                            Label {
                                                text: qsTr("NAVL1_PERIOD")
                                                color: labelColor
                                            }
                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 10

                                                TextField {
                                                    id: navl1PeriodField
                                                    Layout.fillWidth: true
                                                    leftPadding: 10
                                                    topPadding: 15
                                                    Material.accent: ThemeManager.m3["tertiary"]
                                                    selectByMouse: true
                                                    color: ThemeManager.m3["onSurface"]
                                                    selectedTextColor: ThemeManager.m3["onSurface"]
                                                    selectionColor: ThemeManager.m3["inversePrimary"]
                                                    placeholderText: qsTr("Default: 17")
                                                    placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 1.8)
                                                    ToolTip.visible: hovered
                                                    ToolTip.text: qsTr("Time constant of the L1 controller. smaller value, more aggressive turns. Too small value leads to stall.")
                                                    ToolTip.delay: 1000
                                                    ToolTip.timeout: 3000
                                                    background: Rectangle {
                                                        implicitWidth: 250
                                                        implicitHeight: 30
                                                        radius: 10
                                                        color: ThemeManager.m3["surfaceContainerLowest"]
                                                        border { color: borderColor; width: 1 }
                                                    }
                                                }
                                                CustomTextBtn {
                                                    id: navl1PeriodSetBtn
                                                    Layout.preferredHeight: 60
                                                    Layout.preferredWidth: 120
                                                    defaultColor: defaultColorBg
                                                    contentItem: Image {
                                                        id: navl1PeriodSetBtnImage
                                                        anchors.fill: parent
                                                        source: "../../images/svg_images/submitIcon.svg"
                                                        fillMode: Image.PreserveAspectFit
                                                        ColorOverlay {
                                                            anchors.fill: parent
                                                            source: navl1PeriodSetBtnImage
                                                            color: ThemeManager.m3["onSecondaryContainer"]
                                                        }
                                                    }
                                                    onClicked: {
                                                        actionView.setApParamBtnSignal(navl1PeriodField.text, parseFloat(navl1PeriodField.text))
                                                    }
                                                }
                                            }
                                        }
                                        ColumnLayout {
                                            Label {
                                                text: qsTr("LIM_ROLL_CD")
                                                color: labelColor
                                            }
                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 10
                                                TextField {
                                                    id: limRollCDField
                                                    Layout.fillWidth: true
                                                    leftPadding: 10
                                                    topPadding: 15
                                                    Material.accent: ThemeManager.m3["tertiary"]
                                                    selectByMouse: true
                                                    color: ThemeManager.m3["onSurface"]
                                                    selectedTextColor: ThemeManager.m3["onSurface"]
                                                    selectionColor: ThemeManager.m3["inversePrimary"]
                                                    placeholderText: qsTr("Default: 20")
                                                    placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 1.8)
                                                    ToolTip.visible: true
                                                    ToolTip.text: qsTr("Maximum bank angle commanded in modes with stabilized limits. Increase this value for sharper turns.")
                                                    ToolTip.delay: 1000
                                                    ToolTip.timeout: 3000

                                                    background: Rectangle {
                                                        implicitWidth: 250
                                                        implicitHeight: 30
                                                        radius: 10
                                                        color: ThemeManager.m3["surfaceContainerLowest"]
                                                        border { color: borderColor; width: 1 }
                                                    }
                                                }
                                                CustomTextBtn {
                                                    id: limRollCDSetBtn
                                                    Layout.preferredHeight: 60
                                                    Layout.preferredWidth: 120
                                                    defaultColor: defaultColorBg
                                                    contentItem: Image {
                                                        id: limRollCDSetBtnImage
                                                        anchors.fill: parent
                                                        source: "../../images/svg_images/submitIcon.svg"
                                                        fillMode: Image.PreserveAspectFit
                                                        ColorOverlay {
                                                            anchors.fill: parent
                                                            source: limRollCDSetBtnImage
                                                            color: ThemeManager.m3["onSecondaryContainer"]
                                                        }
                                                    }
                                                    onClicked: {
                                                        actionView.setApParamBtnSignal(limRollCDField.text, parseFloat(limRollCDField.text))
                                                    }
                                                }
                                            }
                                        }
                                    }
                                } 

                                 Rectangle {
                                    id: arduplaneParamsSection
                                    Layout.fillWidth: true
                                    Layout.preferredHeight: parent.height
                                    Layout.preferredWidth: 570
                                    color: ThemeManager.m3["surfaceContainerLowest"]
                                    border { color: borderColor; width: 1 }
                                    radius: 5
                                    clip: true

                                    Rectangle {
                                        id: ardupalneParamsHeader
                                        width: parent.width
                                        height: 40
                                        color: labelBgColor
                                        border.color: borderColor
                                        border.width: 1
                                        anchors.left: parent.left
                                        anchors.top: parent.top
                                        clip: true

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
                                                placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 1.8)
                                                placeholderText: qsTr("e.g. LIM_PITCH_MIN")
                                                ToolTip.visible: hovered
                                                ToolTip.text: qsTr("ID or name of the parameters defined in the installed version of arduplane's parameter list.")
                                                ToolTip.delay: 1000
                                                ToolTip.timeout: 3000
                                                background: Rectangle {
                                                    id: apParamIDTextFieldBg
                                                    implicitWidth: 200
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
                                                    placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 1.8)
                                                    placeholderText: qsTr("Enter Value ...")
                                                    ToolTip.visible: hovered
                                                    ToolTip.text: qsTr("Value of the specified parameter in parameter ID text field.")
                                                    ToolTip.delay: 1000
                                                    ToolTip.timeout: 3000
                                                    background: Rectangle {
                                                        id: apParamValueTextFieldBg
                                                        implicitWidth: 150
                                                        implicitHeight: 30
                                                        radius: 10
                                                        color: ThemeManager.m3["surfaceContainerLowest"]
                                                        border { color: borderColor; width: 1 }
                                                    }
                                                }
                                                CustomTextBtn {
                                                    id: allApParamSetBtn
                                                    Layout.preferredHeight: 60
                                                    Layout.preferredWidth: 120
                                                    defaultColor: defaultColorBg
                                                    clip: true
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
                    }
                }
            }
        }
    }
}
