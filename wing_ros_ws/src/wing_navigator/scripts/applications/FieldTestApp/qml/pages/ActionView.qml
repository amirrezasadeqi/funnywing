import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import QtGraphicalEffects 1.15
import "../controls"
import "../theme" 1.0

Item {
    id: actionView
    implicitWidth: 1171
    implicitHeight: 171
    property bool armingState: false
    property string flightMode: qsTr("MANUAL")
    property color labelColor: ThemeManager.m3["onSurface"]
    property color labelBgColor: ThemeManager.m3["surfaceDim"]
    property color borderColor: ThemeManager.m3["outlineVariant"]
    property color actionRectBg: ThemeManager.m3["surfaceContainerLow"]
    property color defaultColorBg: ThemeManager.m3["secondaryContainer"]

    signal armDisarmBtnSignal(bool arming)
    signal modeChangerBtnsSignal(string mode)
    signal testScenarioBtnSignal(int scenarioIdx, bool active)
    signal setSettingsBtnSignal(real waypointRadius, bool local, bool wingAsVirtualCenter)
    signal simpleTrackerBtnsSignal(bool active)
    signal setApParamBtnSignal(string paramName, real paramValue)

    Rectangle {
        id: bg
        color: ThemeManager.m3["surfaceBright"]
        anchors.fill: parent

        Rectangle {
            id: container
            color: "transparent"
            anchors.fill: parent

            ScrollView {
                id: scrollView
                anchors.fill: parent
                contentHeight: 770
                Rectangle {
                    id: armingActionContianer
                    width: 120
                    height: 150
                    color: actionRectBg
                    anchors{
                        left: parent.left
                        leftMargin: 15
                        top: parent.top
                        topMargin: 5
                    }
                    border.color: borderColor
                    border.width: 1
                    radius: 3
                    Item{
                        id: armingActionBtnContainer
                        height: armingBtn.height + disarmBtn.height
                        width: armingBtn.width
                        anchors.centerIn: armingActionContianer
                        CustomTextBtn{
                            id: armingBtn
                            width: 70
                            height: 45
                            btnLabel: qsTr("Arm")
                            defaultColor: defaultColorBg
                            onClicked: {
                                actionView.armDisarmBtnSignal(true)
                            }
                        }
                        CustomTextBtn{
                            id: disarmBtn
                            width: 80
                            height: 45
                            defaultColor: defaultColorBg
                            anchors{
                                top: armingBtn.bottom
                                topMargin: 10
                            }

                            btnLabel: qsTr("Disarm")
                            onClicked: {
                                actionView.armDisarmBtnSignal(false)
                            }
                        }
                    }

                    Rectangle {
                        id: armingActionLabelContainer
                        width: 88
                        height: 22
                        radius: 3
                        color: labelBgColor
                        anchors.left: parent.left
                        anchors.top: parent.top
                        border.color: borderColor
                        Label{
                            id: armingActionLabel
                            text: "Arm/Disarm"
                            color: labelColor
                            anchors.centerIn: parent
                        }
                    }
                }

                Rectangle {
                    id: flightModeActionContainer
                    width: (parent.width - armingActionContianer.width) / 2
                    height: 150
                    color: actionRectBg
                    border.width: 1
                    radius: 3
                    anchors.left: armingActionContianer.right
                    anchors.top: parent.top
                    anchors.leftMargin: 10
                    anchors.topMargin: 5
                    border.color: borderColor
                    Rectangle{
                        id: flightModeActionLabelContaienr
                        width: 150
                        height: 22
                        color: labelBgColor
                        radius: 3
                        anchors.left: parent.left
                        anchors.top: parent.top
                        border.color: borderColor
                        Label{
                            id: flightModeLabel
                            text: "Flight Mode Actions"
                            color: labelColor
                            anchors.centerIn: parent
                        }
                    }

                    CustomComboBox {
                        id: flightModeComboBox
                        anchors {
                            top: flightModeActionLabelContaienr.bottom
                            topMargin: 5
                            horizontalCenter: parent.horizontalCenter
                        }
                    }

                    CustomTextBtn {
                        id: returnToHomeBtn
                        width: 181
                        height: 50
                        anchors.left: flightModeComboBox.left
                        anchors.top: flightModeComboBox.bottom
                        anchors.leftMargin: 0
                        anchors.topMargin: 5
                        btnLabel: "Return to Home"
                        defaultColor: defaultColorBg
                        onClicked: {
                            actionView.modeChangerBtnsSignal(qsTr("RTL"))
                        }
                    }

                    CustomTextBtn {
                        id: activeModeBtn
                        x: 238
                        width: 201
                        height: 50
                        anchors.right: flightModeComboBox.right
                        anchors.top: flightModeComboBox.bottom
                        btnLabel: "Active Mode"
                        anchors.rightMargin: 0
                        anchors.topMargin: 5
                        defaultColor: defaultColorBg
                        onClicked: {
                            actionView.modeChangerBtnsSignal(flightModeComboBox.currentText)
                        }
                    }
                }

                Rectangle {
                    id: testActionContainer
                    height: 342
                    color: actionRectBg
                    border.color: borderColor
                    border.width: 1
                    radius: 3
                    anchors.left: flightModeActionContainer.right
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.rightMargin: 5
                    anchors.leftMargin: 5
                    anchors.topMargin: 5
                    Rectangle{
                        id: testActionLabelContaienr
                        width: 110
                        height: 22
                        color: labelBgColor
                        border.color: borderColor
                        radius: 3
                        border.width: 1

                        anchors {
                            left: parent.left
                            top: parent.top
                        }

                        Label{
                            id: testActionLabel
                            text: "Test Actions"
                            color: labelColor
                            anchors.centerIn: parent
                        }
                    }

                    Rectangle {
                        id: simpleTrackerSettingsGroupContainer
                        x: 334
                        y: 88
                        width: 122
                        height: 96
                        color: "transparent"

                        Label {
                            id: localityExecutionLabel
                            x: 17
                            color: labelColor
                            text: qsTr("Local Execution")
                            anchors.top: setSimpleTrackerSettingsBtn.bottom
                            anchors.topMargin: 5
                            font.pointSize: 9
                        }

                        Switch {
                            id: localityExecutionSwitch
                            x: 17
                            width: 85
                            height: 26
                            Material.accent: ThemeManager.m3["tertiaryContainer"]
                            anchors.top: localityExecutionLabel.bottom
                            anchors.topMargin: 3
                            onToggled: {
                                if (!localityExecutionSwitch.checked){
                                    localityExecutionLabel.text = qsTr("Local Execution");
                                }
                                else{
                                    localityExecutionLabel.text = qsTr("Remote Execution");
                                }
                            }
                        }

                        Label {
                            id: virtualCenterLabel
                            x: 0
                            color: labelColor
                            text: qsTr("Wing as Virtual Center")
                            anchors.top: localityExecutionSwitch.bottom
                            anchors.topMargin: 3
                            font.pointSize: 9
                        }

                        Switch {
                            id: virtualCenterSwitch
                            x: 38
                            width: 45
                            height: 26
                            Material.accent: ThemeManager.m3["tertiaryContainer"]
                            anchors.top: virtualCenterLabel.bottom
                            anchors.topMargin: 3
                            display: AbstractButton.TextOnly
                            onToggled: {
                                if (!virtualCenterSwitch.checked){
                                    virtualCenterLabel.text = qsTr("Wing as Virtual Center");
                                }
                                else{
                                    virtualCenterLabel.text = qsTr("Target as Virtual Center");
                                }
                            }
                        }
                    }
//                    TODO: this part has some unknown bug Actually I think in the Material theme. So I comment
//                          it out to work on it in future. the problem was that the text in the edit part of
//                          the custom text field was out of frame. in future ...
//                    Rectangle {
//                        id: waypointRadiusGroupItem
//                        anchors {
//                            top: testActionLabelContaienr.bottom
//                            topMargin: 10
//                            left: testActionContainer.left
//                            leftMargin: 10
//                        }
//                        color: "transparent"
//                        width: waypointRadiusCustomTextfield.width
//                        height: waypointRadiusCustomTextfield.width + waypointRadiusLabel.height

//                        Label {
//                            id: waypointRadiusLabel
//                            anchors {
//                                top: parent.top
//                                left: parent.left
//                            }
//                            color: labelColor
//                            text: qsTr("Virtual Target Offset")
//                            font.pointSize: 9
//                        }

//                        CustomTextField {
//                            id: waypointRadiusCustomTextfield
//                            width: 300
//                            height: 30
//                            anchors {
//                                top: waypointRadiusLabel.bottom
//                                left: waypointRadiusLabel.left
//                            }
//                            placeholderText: "Enter Virtual Target Offset"
//                            ToolTip.visible: hovered
//                            ToolTip.text: qsTr("Enter a value 5 meters above the wing loiter radius.")
//                            ToolTip.delay: 1000
//                            ToolTip.timeout: 3000
//                        }
//                    }

                    TextField {
                        id: waypointRadiusCustomTextfield
                        leftPadding: 10
                        topPadding: 15
                        Material.accent: ThemeManager.m3["tertiary"]
                        anchors {
                            left: deactiveSimpleTrackerBtn.left
                            verticalCenter: setSimpleTrackerSettingsBtn.verticalCenter
                        }
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
                        background: Rectangle {
                            id: waypointRadiusCustomTextfieldBg
                            implicitWidth: 300
                            implicitHeight: 30
                            radius: 10
                            color: ThemeManager.m3["surfaceContainerLowest"]
                            border {
                                color: borderColor
                                width: 1
                            }
                        }
                    }

                    CustomTextBtn {
                        id: activeSimpleTrackerBtn
                        x: 8
                        width: 288
                        height: 50
                        defaultColor: defaultColorBg
                        anchors.top: deactiveSimpleTrackerBtn.bottom
                        anchors.topMargin: 5
                        font.pointSize: 9
                        btnLabel: "Active Simple Tracker"
                        onClicked: {
                            actionView.simpleTrackerBtnsSignal(true)
                        }
                    }

                    CustomTextBtn {
                        id: deactiveSimpleTrackerBtn
                        x: 8
                        width: 288
                        height: 50
                        defaultColor: defaultColorBg
                        anchors.top: waypointRadiusCustomTextfield.bottom
                        anchors.topMargin: 5
                        font.pointSize: 9
                        btnLabel: "Deactive Simple Tracker"
                        onClicked: {
                            actionView.simpleTrackerBtnsSignal(false)
                        }
                    }

                    Rectangle {
                        id: scenarioSelectionContainer
                        width: 175
                        height: 120
                        anchors {
                            top: activeSimpleTrackerBtn.bottom
                            topMargin: 10
                            left: parent.left
                            leftMargin: 10
                        }
                        color: actionRectBg
                        radius: 3
                        border.width: 1
                        border.color: borderColor

                        Rectangle {
                            id: scenarioLabelContainer
                            width: 100
                            height: 20
                            color: labelBgColor
                            border {
                                color: borderColor
                                width: 1
                            }
                            radius: 3
                            anchors {
                                left: parent.left
                                top: parent.top
                            }
                            Label {
                                id: scenarioSelectionLabel
                                text: qsTr("Test Scenarios")
                                anchors.verticalCenter: parent.verticalCenter
                                font.pointSize: 9
                                anchors.horizontalCenter: parent.horizontalCenter
                                color: labelColor
                            }
                        }

                        CustomTextBtn {
                            id: runTestScenarioBtn
                            width: 200
                            height: 50
                            defaultColor: ThemeManager.m3["primaryContainer"]
                            Material.elevation: 10
                            anchors {
                                left: scenarioSelectionContainer.right
                                leftMargin: 45
                                top: scenarioSelectionContainer.top
                            }
                            btnLabel: "Run Test Scenario"
                            onClicked: {
                                actionView.testScenarioBtnSignal(1, true);
                            }
                        }

                        CustomTextBtn {
                            id: stopScenarioBtn
                            width: 200
                            height: 50
                            anchors {
                                top: runTestScenarioBtn.bottom
                                topMargin: 10
                                left: runTestScenarioBtn.left
                            }
                            defaultColor: ThemeManager.m3["primaryContainer"]
                            btnLabel: "Stop Test Scenario"
                            onClicked: {
                                actionView.testScenarioBtnSignal(1, false);
                            }
                        }

                        RadioButton {
                            id: senarioOneRadioBtn
                            Material.accent: ThemeManager.m3["tertiaryContainer"]
                            anchors {
                                top: parent.top
                                topMargin: 30
                                left: parent.left
                                leftMargin: 5
                            }
                            Text{
                                id: scenarioOneRadioBtnText
                                text: "Scenario 1"
                                color: labelColor
                                anchors{
                                    left: parent.right
                                    verticalCenter: parent.verticalCenter
                                }
                            }
                            checked: true
                        }
                    }

                    CustomTextBtn {
                        id: setSimpleTrackerSettingsBtn
                        width: 161
                        height: 50
                        anchors {
                            top: parent.top
                            topMargin: 30
                            left: waypointRadiusCustomTextfield.right
                            leftMargin: 10
                        }
                        defaultColor: defaultColorBg
                        font.pointSize: 9
                        btnLabel: "Set Settings"
                        onClicked: {
                            let radius = (waypointRadiusCustomTextfield.text.length === 0) ? 120.0 : parseFloat(waypointRadiusCustomTextfield.text);
                            actionView.setSettingsBtnSignal(radius, !localityExecutionSwitch.checked, !virtualCenterSwitch.checked)
                        }
                    }
                }

                Rectangle {
                    id: arduplaneConfigContainer
                    height: 600
                    color: actionRectBg
                    anchors{
                        left: parent.left
                        leftMargin: 15
                        top: armingActionContianer.bottom
                        topMargin: 5
                        right: testActionContainer.left
                        rightMargin: 5
                    }
                    border.color: borderColor
                    border.width: 1
                    radius: 3

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
                            Label{
                                id: compLabel
                                color: labelColor
                                anchors{
                                    top: parent.top
                                    left: parant.left
                                }
                                text: bg.compLabelText
                            }
                            TextField {
                                id: compTextField
                                leftPadding: 10
                                topPadding: 15
                                Material.accent: ThemeManager.m3["tertiary"]
                                selectByMouse: true
                                color: ThemeManager.m3["onSurface"]
                                selectedTextColor: ThemeManager.m3["onSurface"]
                                selectionColor: ThemeManager.m3["inversePrimary"]
                                anchors {
                                    left: parent.left
                                    top: compLabel.bottom
                                    topMargin: 5
                                }
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
                                    border {
                                        color: borderColor
                                        width: 1
                                    }
                                }
                            }
                            CustomTextBtn {
                                id: compSetBtn
                                width: 50
                                height: 50
                                defaultColor: defaultColorBg
                                anchors {
                                    verticalCenter: compTextField.verticalCenter
                                    left: compTextField.right
                                    leftMargin: 5
                                }
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

                    Rectangle {
                        id: arduplaneConfigLabelContainer
                        width: 190
                        height: 22
                        radius: 3
                        color: labelBgColor
                        border {
                            color: borderColor
                            width: 1
                        }
                        anchors {
                            left: parent.left
                            top: parent.top
                        }
                        Label{
                            id: arduplaneConfigLabel
                            text: "ArduPlane Configurations"
                            color: labelColor
                            anchors.centerIn: parent
                        }
                    }

                    Rectangle {
                        id: flightRadiusConfigContainer
                        y: 35
                        height: 200
                        color: actionRectBg
                        radius: 3
                        border {
                           color: borderColor
                            width: 1
                        }
                        anchors {
                            left: parent.left
                            right: parent.right
                            rightMargin: 10
                            leftMargin: 10
                        }
                        Rectangle {
                            id: flightRadiusConfigLabelContainer
                            width: 141
                            height: 20
                            color: labelBgColor
                            border {
                                color: borderColor
                                width: 1
                            }
                            radius: 3
                            anchors {
                                left: parent.left
                                top: parent.top
                            }
                            Label {
                                id: flightRadiusConfigLabel
                                color: labelColor
                                text: qsTr("Flight Radius Config")
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.verticalCenterOffset: 1
                                anchors.leftMargin: 3
                            }
                        }
                        Loader{
                            id: wpLoiterRadSubLoader
                            sourceComponent: submitControlComp
                            anchors{
                                top: flightRadiusConfigLabelContainer.bottom
                                topMargin: 5
                                left: parent.left
                                leftMargin: 10
                            }
                            onLoaded: {
                                item.paramID = qsTr("WP_LOITER_RAD")
                                item.compLabelText = qsTr("Waypoint Loiter Radius")
                                item.compTextFieldPlaceHolderText = qsTr("Default: 120")
                                item.compTextFieldTooltipText = qsTr("The radius that wing tries to hold when turning around waypoint.")
                            }
                        }
                        Loader{
                            id: wpRadSubLoader
                            sourceComponent: submitControlComp
                            anchors{
                                top: wpLoiterRadSubLoader.bottom
                                topMargin: 10
                                left: parent.left
                                leftMargin: 10
                            }
                            onLoaded: {
                                item.paramID = qsTr("WP_RADIUS")
                                item.compLabelText = qsTr("Waypoint Radius")
                                item.compTextFieldPlaceHolderText = qsTr("Default: 120")
                                item.compTextFieldTooltipText = qsTr("The radius within that the waypoint is considered to be reached.")
                            }
                        }
                    }
                    Rectangle {
                        id: controllerParamsContainer
                        y: 35
                        height: 200
                        color: actionRectBg
                        radius: 3
                        border {
                            color: borderColor
                            width: 1
                        }
                        anchors {
                            left: parent.left
                            right: parent.right
                            top: flightRadiusConfigContainer.bottom
                            rightMargin: 10
                            leftMargin: 10
                            topMargin: 5
                        }
                        Rectangle {
                            id: controllerParamsLabelContainer
                            width: 141
                            height: 20
                            color: labelBgColor
                            radius: 3
                            border {
                                color: borderColor
                                width: 1
                            }
                            anchors {
                                left: parent.left
                                top: parent.top
                            }
                            Label {
                                id: controllerParamsLabel
                                color: labelColor
                                text: qsTr("Controller Params")
                                anchors {
                                    verticalCenter: parent.verticalCenter
                                    left: parent.left
                                    verticalCenterOffset: 1
                                    leftMargin: 3
                                }
                            }
                        }
                        Loader {
                            id: navL1PeriodLoader
                            sourceComponent: submitControlComp
                            anchors{
                                top: controllerParamsLabelContainer.bottom
                                topMargin: 5
                                left: parent.left
                                leftMargin: 10
                            }
                            onLoaded: {
                                item.paramID = qsTr("NAVL1_PERIOD")
                                item.compLabelText = qsTr("NAVL1_PERIOD")
                                item.compTextFieldPlaceHolderText = qsTr("Default: 17")
                                item.compTextFieldTooltipText = qsTr("Time constant of the L1 controller. Smaller value, more aggressive turns. Too small[10] value leads to stall.")
                            }
                        }
                        Loader {
                            id: limRollCdLoader
                            sourceComponent: submitControlComp
                            anchors{
                                top: navL1PeriodLoader.bottom
                                topMargin: 5
                                left: parent.left
                                leftMargin: 10
                            }
                            onLoaded: {
                                item.paramID = qsTr("LIM_ROLL_CD")
                                item.compLabelText = qsTr("LIM_ROLL_CD")
                                item.compTextFieldPlaceHolderText = qsTr("Default: 20")
                                item.compTextFieldTooltipText = qsTr("Maximum bank angle commanded in modes with stabilized limits. Increase this value for sharper turns.")
                            }
                        }
                    }
                    Rectangle {
                        id: allApParamsConfigContainer
                        y: 35
                        height: 130
                        color: actionRectBg
                        radius: 3
                        border {
                            color: borderColor
                            width: 1
                        }
                        anchors {
                            left: parent.left
                            right: parent.right
                            top: controllerParamsContainer.bottom
                            rightMargin: 10
                            leftMargin: 10
                            topMargin: 5
                        }
                        Rectangle {
                            id: allApParamsConfigLabelContainer
                            width: 180
                            height: 20
                            color: labelBgColor
                            radius: 3
                            anchors {
                                left: parent.left
                                top: parent.top
                            }
                            border {
                                color: borderColor
                                width: 1
                            }
                            Label {
                                id: allApParamsConfigLabel
                                color: labelColor
                                text: qsTr("ArduPlane Params Config")
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.verticalCenterOffset: 1
                                anchors.leftMargin: 3
                            }
                        }
                        Rectangle {
                            id: allApParamsSubmitterBg
                            implicitWidth: apParamIDTextField.width + apParamValueTextField.width + allApParamSetBtn.width + 5
                            implicitHeight: apParamIDTextField.height + apParamLabel.height + 5
                            anchors {
                                top: allApParamsConfigLabelContainer.bottom
                                topMargin: 5
                                left: parent.left
                                leftMargin: 10
                            }
                            color: "transparent"
                            Label{
                                id: apParamLabel
                                color: labelColor
                                anchors{
                                    top: parent.top
                                    left: parant.left
                                }
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

                                anchors {
                                    left: parent.left
                                    top: apParamLabel.bottom
                                    topMargin: 5
                                }

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
                                    border {
                                        color: borderColor
                                        width: 1
                                    }
                                }
                            }
                            Label{
                                id: apParamValueLabel
                                color: labelColor
                                anchors{
                                    top: parent.top
                                    left: apParamIDTextField.right
                                    leftMargin: 10
                                }
                                text: qsTr("ArduPlane Parameter Value")
                            }

                            TextField {
                                id: apParamValueTextField
                                leftPadding: 10
                                topPadding: 15
                                Material.accent: ThemeManager.m3["tertiary"]
                                selectByMouse: true
                                color: ThemeManager.m3["onSurface"]
                                selectedTextColor: ThemeManager.m3["onSurface"]
                                selectionColor: ThemeManager.m3["inversePrimary"]
                                placeholderTextColor: ThemeManager.m3["onSurface"]
                                placeholderText: qsTr("Enter Value ...")

                                anchors {
                                    left: apParamIDTextField.right
                                    top: apParamValueLabel.bottom
                                    leftMargin: 10
                                    topMargin: 5
                                }

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
                                    border {
                                        color: borderColor
                                        width: 1
                                    }
                                }
                            }

                            CustomTextBtn {
                                id: allApParamSetBtn
                                width: 50
                                height: 50
                                defaultColor: defaultColorBg
                                anchors {
                                    verticalCenter: apParamValueTextField.verticalCenter
                                    left: apParamValueTextField.right
                                    leftMargin: 5
                                }
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


