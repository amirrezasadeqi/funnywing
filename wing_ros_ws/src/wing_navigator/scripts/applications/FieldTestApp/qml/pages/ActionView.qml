import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import QtGraphicalEffects 1.15
import "../controls"
import "../theme" 1.0

Item {
    id: actionView

    property bool armingState: false
    property string flightMode: qsTr("MANUAL")
    property color labelColor: ThemeManager.m3["onSurface"]
    property color labelBgColor: ThemeManager.m3["surfaceDim"]
    property color borderColor: ThemeManager.m3["outlineVariant"]
    property color actionRectBg: ThemeManager.m3["surfaceContainerLow"]
    property color defaultColorBg: ThemeManager.m3["secondaryContainer"]

    signal armDisarmBtnSignal(bool arming)
    signal modeChangerBtnsSignal(string mode)
    signal simpleTrackerBtnsSignal(bool active)

    property var wingAtt: {'roll': 0, 'pitch': 0, 'yaw': 0}

    Rectangle {
        anchors.fill: parent
        color: ThemeManager.m3["surfaceBright"]
        clip: true

        Item {
            id: fitWrapper
            anchors.fill: parent

            Item {
                id: content
                width: 2500
                height: 320
                anchors.centerIn: parent

                readonly property real s: Math.min(
                    fitWrapper.width /width,
                    fitWrapper.height / height,
                    1.0
                )
                scale: s

                RowLayout {
                    id: rowContainer
                    spacing: 30
                    anchors.fill: parent
                    anchors.margins: 15

                    Item { Layout.preferredWidth: 10 }

                    Rectangle {
                        Layout.preferredWidth: 400
                        Layout.preferredHeight: 240
                        color: actionRectBg
                        border.color: borderColor
                        radius: 6

                        ColumnLayout {
                            anchors.centerIn: parent
                            spacing: 15
                            Item {
                                Layout.preferredHeight: 30
                            }
                        CustomTextBtn {
                                Layout.preferredWidth: 250
                                Layout.preferredHeight: 80
                                btnLabel: qsTr("Arm")
                                defaultColor: defaultColorBg
                                onClicked: actionView.armDisarmBtnSignal(true)
                            }
                            CustomTextBtn {
                                Layout.preferredWidth: 250
                                Layout.preferredHeight: 80
                                btnLabel: qsTr("Disarm")
                                defaultColor: defaultColorBg
                                onClicked: actionView.armDisarmBtnSignal(false)
                            }
                        }

                        Rectangle {
                            width: parent.width
                            height: 50
                            radius: 3
                            color: labelBgColor
                            anchors.left: parent.left
                            anchors.top: parent.top
                            border.color: borderColor
                            Label {
                                text: "Arm/Disarm"
                                color: labelColor
                                anchors.centerIn: parent
                                font.pixelSize: 25
                            }
                        }
                    }

                    Rectangle {
                        Layout.preferredWidth: 440
                        Layout.preferredHeight: 240
                        color: actionRectBg
                        border.color: borderColor
                        radius: 6

                        ColumnLayout {
                            anchors.centerIn: parent
                            spacing: 10
                            Item {
                                Layout.preferredHeight: 30
                            }
                            CustomComboBox {
                                id: flightModeComboBox
                                Layout.preferredWidth: 410
                                Layout.preferredHeight: 60
                                font.pixelSize: 24
                            }

                            RowLayout {
                                spacing: 10
                                CustomTextBtn {
                                    Layout.preferredWidth: 200
                                    Layout.preferredHeight: 80
                                    btnLabel: "Return to Home"
                                    defaultColor: defaultColorBg
                                    onClicked: actionView.modeChangerBtnsSignal("RTL")
                                }
                                CustomTextBtn {
                                    Layout.preferredWidth: 200
                                    Layout.preferredHeight: 80
                                    btnLabel: "Active Mode"
                                    defaultColor: defaultColorBg
                                    onClicked: actionView.modeChangerBtnsSignal(flightModeComboBox.currentText)
                                }
                            }
                        }

                        Rectangle {
                            width: parent.width
                            height: 50
                            radius: 3
                            color: labelBgColor
                            anchors.left: parent.left
                            anchors.top: parent.top
                            border.color: borderColor
                            Label {
                                text: "Flight Mode Actions"
                                color: labelColor
                                anchors.centerIn: parent
                                font.pixelSize: 25
                            }
                        }
                    }

                    Rectangle {
                        Layout.preferredWidth: 400
                        Layout.preferredHeight: 240
                        color: actionRectBg
                        border.color: borderColor
                        radius: 6

                        ColumnLayout {
                            anchors.centerIn: parent
                            spacing: 10
                            Item {
                                Layout.preferredHeight: 30
                            }
                            CustomTextBtn {
                                Layout.preferredWidth: 295
                                Layout.preferredHeight: 80
                                btnLabel: "Active Simple Tracker"
                                defaultColor: defaultColorBg
                                onClicked: actionView.simpleTrackerBtnsSignal(true)
                            }
                            CustomTextBtn {
                                Layout.preferredWidth: 295
                                Layout.preferredHeight: 80
                                btnLabel: "Deactive Simple Tracker"
                                defaultColor: defaultColorBg
                                onClicked: actionView.simpleTrackerBtnsSignal(false)
                            }
                        }

                        Rectangle {
                            width: parent.width
                            height: 50
                            radius: 3
                            color: labelBgColor
                            anchors.left: parent.left
                            anchors.top: parent.top
                            border.color: borderColor
                            Label {
                                text: "Simple Tracker Actions"
                                color: labelColor
                                anchors.centerIn: parent
                                font.pixelSize: 25
                            }
                        }
                    }

                    Item { Layout.preferredWidth: 120 }

                    Item {
                        Layout.preferredWidth: 300
                        Layout.preferredHeight: 300
                        Layout.alignment: Qt.AlignHCenter
                        z: 2
    
                        Rectangle {
                            anchors.centerIn: parent
                            width: turnCoordinatorImage.width * 0.75
                            height: width
                            color: "white"
                            radius: width / 2
                            z: 0
                        }

                        Image {
                            id: turnCoordinatorImage
                            anchors.fill: parent
                            source: "turnCoordinator.png"
                            fillMode: Image.PreserveAspectFit
                            z: 1
                        }

                        Image {
                            anchors.centerIn: parent
                            source: "uavFront.png"
                            fillMode: Image.PreserveAspectFit
                            width: parent.width * 0.5
                            height: width
                            z: 2
                            rotation: actionView.wingAtt.roll
                        }           
                    }


                    Item {
                        Layout.preferredWidth: 300
                        Layout.preferredHeight: 300
                        Layout.alignment: Qt.AlignHCenter

                        Image {
                            anchors.fill: parent
                            source: "yawCompass.png"
                            fillMode: Image.PreserveAspectFit
                        }
                        Image {
                            anchors.centerIn: parent
                            width: parent.width * 0.4
                            source: "uavUp.png"
                            fillMode: Image.PreserveAspectFit
                            rotation: actionView.wingAtt.yaw
                        }
                    }

                    Item {
                        Layout.preferredWidth: 300
                        Layout.preferredHeight: 300
                        Layout.alignment: Qt.AlignHCenter

                        Image {
                            anchors.fill: parent
                            source: "pitchIndicator.png"
                            fillMode: Image.PreserveAspectFit
                        }
                        Image {
                            anchors.centerIn: parent
                            width: parent.width * 0.5
                            source: "uav.png"
                            fillMode: Image.PreserveAspectFit
                            rotation: actionView.wingAtt.pitch
                        }
                    }
                }
            }
        }
    }
}