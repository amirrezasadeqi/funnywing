import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../controls"

Item {
    id: stateActionView
    implicitWidth: 1171
    implicitHeight: 171
    property bool armingState: false
    property string flightMode: qsTr("MANUAL")

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

                ColumnLayout {
                    id: columnLayout
                    spacing: 20
                    Rectangle {
                        id: systemArmingActionContianer
                        Layout.preferredWidth: scrollView.width
                        height: 100
                        color: "transparent"
                        CustomGroupBox{
                            id: systemArmingActionGroupBox
                            anchors.fill: parent
                            title: "System Arming Actions"
                            contentItem: Rectangle{
                                id: systemArmingActionControlContainer
                                anchors.fill: parent
                                color: "transparent"
                                RowLayout{
                                    id: armingActionsRowLayout
                                    anchors.verticalCenter: parent.verticalCenter
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    spacing: 100
                                    CustomTextBtn{
                                        id: armingBtn
                                        btnLabel: qsTr("Arm")
                                        Layout.topMargin: (systemArmingActionControlContainer.height - height) / 2
                                        onClicked: {
                                            stateActionView.armingState = true
                                        }
                                    }
                                    CustomTextBtn{
                                        id: disarmBtn
                                        btnLabel: qsTr("Disarm")
                                        Layout.topMargin: (systemArmingActionControlContainer.height - height) / 2
                                        onClicked: {
                                            stateActionView.armingState = false
                                        }
                                    }
                                }
                            }
                        }
                    }

                    Rectangle {
                        id: flightModeActionContainer
                        Layout.preferredWidth: scrollView.width
                        height: 200
                        color: "transparent"
                        CustomGroupBox{
                            id: flightModeActionGroupBox
                            anchors.fill: parent
                            title: "Flight Mode Actions"
                            contentItem: Rectangle{
                                id: flightModeActionControlContainer
                                anchors.fill: parent
                                color: "transparent"
                                ColumnLayout{
                                    id: flightModeColumnLayout
                                    anchors.verticalCenter: parent.verticalCenter
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    RowLayout{
                                        id: flightModeRowLayout
                                        anchors.verticalCenter: parent.verticalCenter
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        spacing: 100
                                        ComboBox{
                                            id: flightModeComboBox
                                            Layout.preferredHeight: activeModeBtn.height
                                            Layout.preferredWidth: activeModeBtn.width
                                            wheelEnabled: true
                                            editable: true
                                            model: ["MANUAL", "STABILIZE", "AUTO", "GUIDED", "RTL", "FBWA"]
                                        }
                                        CustomTextBtn{
                                            id: activeModeBtn
                                            btnLabel: qsTr("Active Mode")
                                            onClicked: {
                                                stateActionView.flightMode = flightModeComboBox.currentText
                                            }
                                        }
                                    }
                                    CustomTextBtn{
                                        id: rtlBtn
                                        anchors.top: flightModeRowLayout.bottom
                                        anchors.topMargin: 10
                                        Layout.alignment: Qt.AlignHCenter
                                        btnLabel: qsTr("Return to Home")
                                        onClicked: {
                                            stateActionView.flightMode = qsTr("RTL")
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
