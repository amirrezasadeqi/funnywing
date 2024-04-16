import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../controls"

Item {
    id: actionView
    implicitWidth: 1171
    implicitHeight: 171
    property bool armingState: false
    property string flightMode: qsTr("MANUAL")

    signal armDisarmBtnSignal(bool arming)
    signal modeChangerBtnsSignal(string mode)
    signal testScenarioBtnSignal(int scenarioIdx, bool active)
    signal setSettingsBtnSignal(real waypointRadius, bool local, bool wingAsVirtualCenter)
    signal simpleTrackerBtnsSignal(bool active)
    signal setApParamBtnSignal(string paramName, real paramValue)

    Rectangle {
        id: bg
        color: "#2a2a2a"
        anchors.fill: parent

        Rectangle {
            id: container
            color: "#00ffffff"
            anchors.fill: parent

            ScrollView {
                id: scrollView
                anchors.fill: parent
                contentHeight: 630
                Rectangle {
                    id: armingActionContianer
                    width: 120
                    height: 150
                    color: "transparent"
                    anchors{
                        left: parent.left
                        leftMargin: 15
                        top: parent.top
                        topMargin: 5
                    }
                    border.color: "green"
                    border.width: 3
                    radius: 5
                    Item{
                        id: armingActionBtnContainer
                        height: armingBtn.height + disarmBtn.height
                        width: armingBtn.width
                        anchors.centerIn: armingActionContianer
                        CustomTextBtn{
                            id: armingBtn
                            width: 90
                            height: 30
                            btnLabel: qsTr("Arm")
                            onClicked: {
                                actionView.armDisarmBtnSignal(true)
                            }
                        }
                        CustomTextBtn{
                            id: disarmBtn
                            width: 90
                            height: 30
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
                        radius: 5
                        color: "green"
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.leftMargin: 0
                        anchors.topMargin: 0
                        Label{
                            id: armingActionLabel
                            text: "Arm/Disarm"
                            color: "white"
                            anchors.centerIn: parent
                        }
                    }
                }

                Rectangle {
                    id: flightModeActionContainer
                    width: (parent.width - armingActionContianer.width) / 2
                    height: 150
                    color: "transparent"
                    border.color: "green"
                    border.width: 3
                    radius: 5
                    anchors.left: armingActionContianer.right
                    anchors.top: parent.top
                    anchors.leftMargin: 10
                    anchors.topMargin: 5
                    Rectangle{
                        id: flightModeActionLabelContaienr
                        width: 150
                        height: 22
                        color: "green"
                        radius: 5
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.leftMargin: 0
                        anchors.topMargin: 0
                        Label{
                            id: flightModeLabel
                            text: "Flight Mode Actions"
                            color: "white"
                            anchors.centerIn: parent
                        }
                    }
                    ComboBox{
                        id: flightModeComboBox
                        width: 400
                        height: 50
                        anchors.top: flightModeActionLabelContaienr.bottom
                        anchors.topMargin: 5
                        anchors.horizontalCenter: parent.horizontalCenter
                        wheelEnabled: true
                        editable: true
                        model: ["MANUAL", "STABILIZE", "AUTO", "GUIDED", "RTL", "FBWA"]
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
                        onClicked: {
                            actionView.modeChangerBtnsSignal(flightModeComboBox.currentText)
                        }
                    }
                }

                Rectangle {
                    id: testActionContainer
                    height: 342
                    color: "transparent"
                    border.color: "green"
                    border.width: 3
                    radius: 5
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
                        color: "green"
                        radius: 5
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.leftMargin: 0
                        anchors.topMargin: 0
                        Label{
                            id: testActionLabel
                            text: "Test Actions"
                            color: "white"
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
                            y: 0
                            color: "#ffffff"
                            text: qsTr("Local Execution")
                            font.pointSize: 9
                        }

                        Switch {
                            id: localityExecutionSwitch
                            x: 17
                            y: 21
                            width: 85
                            height: 26
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
                            y: 53
                            color: "#feffff"
                            text: qsTr("Wing as Virtual Center")
                            font.pointSize: 9
                        }

                        Switch {
                            id: virtualCenterSwitch
                            x: 38
                            y: 67
                            width: 45
                            height: 26
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

                    Rectangle {
                        id: waypointRadiusGroupItem
                        x: 32
                        y: 30
                        color: "transparent"
                        width: waypointRadiusCustomTextfield.width
                        height: waypointRadiusCustomTextfield.width + waypointRadiusLabel.height

                        CustomTextField {
                            id: waypointRadiusCustomTextfield
                            x: -20
                            y: 21
                            width: 284
                            height: 31
                            placeholderText: "Enter Virtual Target Offset"
                            ToolTip.visible: hovered
                            ToolTip.text: qsTr("Enter a value 5 meters above the wing loiter radius.")
                            ToolTip.delay: 1000
                            ToolTip.timeout: 3000
                        }

                        Label {
                            id: waypointRadiusLabel
                            x: -20
                            y: 0
                            color: "#ffffff"
                            text: qsTr("Virtual Target Offset")
                            font.pointSize: 9
                        }
                    }

                    CustomTextBtn {
                        id: activeSimpleTrackerBtn
                        x: 8
                        y: 129
                        width: 288
                        height: 35
                        font.pointSize: 9
                        btnLabel: "Active Simple Tracker"
                        onClicked: {
                            actionView.simpleTrackerBtnsSignal(true)
                        }
                    }

                    CustomTextBtn {
                        id: deactiveSimpleTrackerBtn
                        x: 8
                        y: 88
                        width: 288
                        height: 35
                        font.pointSize: 9
                        btnLabel: "Deactive Simple Tracker"
                        onClicked: {
                            actionView.simpleTrackerBtnsSignal(false)
                        }
                    }

                    Rectangle {
                        id: scenarioSelectionContainer
                        x: 8
                        y: 188
                        width: 175
                        height: 121
                        color: "#00ffffff"
                        radius: 5
                        border.width: 3
                        border.color: "green"

                        Rectangle {
                            id: scenarioLabelContainer
                            width: 100
                            height: 20
                            color: "green"
                            radius: 5
                            anchors.left: parent.left
                            anchors.top: parent.top

                            Label {
                                id: scenarioSelectionLabel
                                text: qsTr("Test Scenarios")
                                anchors.verticalCenter: parent.verticalCenter
                                font.pointSize: 9
                                anchors.horizontalCenter: parent.horizontalCenter
                                color: "white"
                            }
                        }

                        CustomTextBtn {
                            id: runTestScenarioBtn
                            x: 217
                            y: 13
                            width: 205
                            height: 35
                            btnLabel: "Run Test Scenario"
                            onClicked: {
                                actionView.testScenarioBtnSignal(1, true);
                            }
                        }

                        CustomTextBtn {
                            id: stopScenarioBtn
                            x: 217
                            y: 64
                            width: 205
                            height: 34
                            btnLabel: "Stop Test Scenario"
                            onClicked: {
                                actionView.testScenarioBtnSignal(1, false);
                            }
                        }

                        RadioButton {
                            id: senarioOneRadioBtn
                            x: 8
                            y: 26
                            Text{
                                id: scenarioOneRadioBtnText
                                text: "Scenario 1"
                                color: "white"
                                anchors{
                                    left: parent.right
                                    verticalCenter: parent.verticalCenter
                                }
                            }

                            checked: true
                        }
                    }

                    CustomTextBtn {
                        id: customTextBtn
                        x: 314
                        y: 51
                        width: 161
                        height: 31
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
                    height: 463
                    color: "transparent"
                    anchors{
                        left: parent.left
                        leftMargin: 15
                        top: armingActionContianer.bottom
                        topMargin: 5
                        right: testActionContainer.left
                        rightMargin: 5
                    }
                    border.color: "green"
                    border.width: 3
                    radius: 5

                    Component {
                        id: submitControlComp
                        Rectangle {
                            id: bg
                            implicitWidth: compTextField.width + compSetBtn.width + 5
                            implicitHeight: compLabel.height + compTextField.height + 5
                            color: "#00ffffff"
                            border.color: "#00000000"
                            property string compLabelText: qsTr("LIM_ROLL_CD")
                            property string paramID: qsTr("LIM_ROLL_CD")
                            property string compTextFieldPlaceHolderText: qsTr("Default: 20")
                            property string compTextFieldTooltipText: qsTr("Some Explanations about this input.")
                            property string compSetBtnImageSrc: "../../images/svg_images/submitIcon.svg"
                            Label{
                                id: compLabel
                                color: "white"
                                anchors{
                                    top: parent.top
                                    left: parant.left
                                }
                                text: bg.compLabelText
                            }
                            CustomTextField {
                                id: compTextField
                                width: 250
                                height: 30
                                anchors.left: parent.left
                                anchors.top: compLabel.bottom
                                placeholderText: bg.compTextFieldPlaceHolderText
                                anchors.topMargin: 5
                                anchors.leftMargin: 0
                                ToolTip.visible: hovered
                                ToolTip.text: bg.compTextFieldTooltipText
                                ToolTip.delay: 1000
                                ToolTip.timeout: 3000
                            }
                            CustomTextBtn {
                                id: compSetBtn
                                width: 50
                                height: 27
                                anchors.verticalCenter: compTextField.verticalCenter
                                anchors.left: compTextField.right
                                anchors.leftMargin: 5
                                contentItem: Image {
                                    id: compSetBtnImage
                                    anchors.fill: parent
                                    source: bg.compSetBtnImageSrc
                                    fillMode: Image.PreserveAspectFit
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
                        radius: 5
                        color: "green"
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.leftMargin: 0
                        anchors.topMargin: 0
                        Label{
                            id: arduplaneConfigLabel
                            text: "ArduPlane Configurations"
                            color: "white"
                            anchors.centerIn: parent
                        }
                    }

                    Rectangle {
                        id: flightRadiusConfigContainer
                        y: 35
                        height: 151
                        color: "#00ffffff"
                        radius: 5
                        border.color: "green"
                        border.width: 3
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.rightMargin: 10
                        anchors.leftMargin: 10

                        Rectangle {
                            id: flightRadiusConfigLabelContainer
                            width: 141
                            height: 20
                            color: "green"
                            radius: 5
                            anchors.left: parent.left
                            anchors.top: parent.top
                            anchors.leftMargin: 0
                            anchors.topMargin: 0

                            Label {
                                id: flightRadiusConfigLabel
                                color: "white"
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
                        height: 150
                        color: "#00ffffff"
                        radius: 5
                        border.color: "green"
                        border.width: 3
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: flightRadiusConfigContainer.bottom
                        anchors.rightMargin: 10
                        anchors.leftMargin: 10
                        anchors.topMargin: 5

                        Rectangle {
                            id: controllerParamsLabelContainer
                            width: 141
                            height: 20
                            color: "green"
                            radius: 5
                            anchors.left: parent.left
                            anchors.top: parent.top
                            anchors.leftMargin: 0
                            anchors.topMargin: 0

                            Label {
                                id: controllerParamsLabel
                                color: "white"
                                text: qsTr("Controller Params")
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.verticalCenterOffset: 1
                                anchors.leftMargin: 3
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
                        height: 93
                        color: "#00ffffff"
                        radius: 5
                        border.color: "green"
                        border.width: 3
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: controllerParamsContainer.bottom
                        anchors.rightMargin: 10
                        anchors.leftMargin: 10
                        anchors.topMargin: 5

                        Rectangle {
                            id: allApParamsConfigLabelContainer
                            width: 180
                            height: 20
                            color: "green"
                            radius: 5
                            anchors.left: parent.left
                            anchors.top: parent.top
                            anchors.leftMargin: 0
                            anchors.topMargin: 0

                            Label {
                                id: allApParamsConfigLabel
                                color: "white"
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
                            color: "#00ffffff"
                            border.color: "#00000000"
                            Label{
                                id: apParamLabel
                                color: "white"
                                anchors{
                                    top: parent.top
                                    left: parant.left
                                }
                                text: qsTr("ArduPlane Parameter ID")
                            }
                            CustomTextField {
                                id: apParamIDTextField
                                width: 250
                                height: 30
                                anchors.left: parent.left
                                anchors.top: apParamLabel.bottom
                                placeholderText: qsTr("e.g. LIM_PITCH_MIN")
                                anchors.topMargin: 5
                                anchors.leftMargin: 0
                                ToolTip.visible: hovered
                                ToolTip.text: qsTr("ID or name of the parameters defined in the installed version of arduplane's parameter list.")
                                ToolTip.delay: 1000
                                ToolTip.timeout: 3000
                            }
                            Label{
                                id: apParamValueLabel
                                color: "white"
                                anchors{
                                    top: parent.top
                                    left: apParamIDTextField.right
                                    leftMargin: 10
                                }
                                text: qsTr("ArduPlane Parameter Value")
                            }
                            CustomTextField {
                                id: apParamValueTextField
                                width: 200
                                height: 30
                                anchors.left: apParamIDTextField.right
                                anchors.top: apParamValueLabel.bottom
                                anchors.leftMargin: 10
                                anchors.topMargin: 5
                                placeholderText: qsTr("Enter Value ...")
                                ToolTip.visible: hovered
                                ToolTip.text: qsTr("Value of the specified parameter in parameter ID text field.")
                                ToolTip.delay: 1000
                                ToolTip.timeout: 3000
                            }
                            CustomTextBtn {
                                id: allApParamSetBtn
                                width: 50
                                height: 27
                                anchors.verticalCenter: apParamValueTextField.verticalCenter
                                anchors.left: apParamValueTextField.right
                                anchors.leftMargin: 5
                                contentItem: Image {
                                    id: allApParamSetBtnImage
                                    anchors.fill: parent
                                    source: "../../images/svg_images/submitIcon.svg"
                                    fillMode: Image.PreserveAspectFit
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


