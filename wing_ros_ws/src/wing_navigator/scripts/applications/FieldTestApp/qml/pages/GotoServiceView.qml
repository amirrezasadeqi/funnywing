import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import "../controls"
import "../theme" 1.0

Item {
    id: gotoServiceView
    implicitWidth: 1171
    implicitHeight: 171

    // To send GPS location to FieldTestApp.qml
    signal goToSignal(real lat, real lon, real alt)

    property color labelColor: ThemeManager.m3["onSurface"]

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

                ColumnLayout {
                    id: columnLayout
                    spacing: 20
                    Rectangle {
                        id: gotoServiceContainer
                        Layout.preferredWidth: scrollView.width
                        height: 200
                        color: "transparent"
                        CustomGroupBox{
                            id: gotoServiceGroupBox
                            anchors.fill: parent
                            title: "Goto Service Actions"
                            contentItem: Rectangle{
                                id: gotoServiceControlContainer
                                anchors.fill: parent
                                color: "transparent"
                                ColumnLayout{
                                    id: gotoServiceColumnLayout
                                    anchors.fill: parent
                                    anchors.leftMargin: 20
                                    anchors.rightMargin: 20
                                    anchors.top: parent.top
                                    anchors.topMargin: parent.height * 0.2
                                    clip: true
                                    RowLayout{
                                        id: gpsLocationRowLayout
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        spacing: 10
                                        Label{
                                            id: latLabel
                                            text: qsTr("Latitude: ")
                                            color: labelColor
                                            font.pointSize: 14
                                        }
                                        Rectangle{
                                            id: latRect
                                            Layout.preferredWidth: gotoServiceColumnLayout.width / 6
                                            Layout.preferredHeight: 30
                                            color: "transparent"
                                            TextField {
                                                id: latTextField
                                                anchors {
                                                    verticalCenter: parent.verticalCenter
                                                    verticalCenterOffset: -2
                                                }
                                                leftPadding: 10
                                                topPadding: 15
                                                Material.accent: ThemeManager.m3["tertiary"]
                                                placeholderText: "Enter Latitude"
                                                color: ThemeManager.m3["onSurface"]
                                                placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 2)
                                                selectByMouse: true
                                                selectedTextColor: ThemeManager.m3["onSurface"]
                                                selectionColor: ThemeManager.m3["inversePrimary"]
                                                background: Rectangle {
                                                    id: latTextfieldBg
                                                    implicitWidth: latRect.width
                                                    implicitHeight: latRect.height
                                                    radius: 10
                                                    color: ThemeManager.m3["surfaceContainerLowest"]
                                                    border {
                                                        color: borderColor
                                                        width: 1
                                                    }
                                                }
                                            }
                                        }
                                        Label{
                                            id: lonLabel
                                            text: qsTr("Longitude: ")
                                            color: labelColor
                                            font.pointSize: 14
                                        }
                                        Rectangle{
                                            id: lonRect
                                            Layout.preferredWidth: gotoServiceColumnLayout.width / 6
                                            Layout.preferredHeight: 30
                                            color: "transparent"
                                            TextField {
                                                id: lonTextField
                                                anchors {
                                                    verticalCenter: parent.verticalCenter
                                                    verticalCenterOffset: -2
                                                }
                                                leftPadding: 10
                                                topPadding: 15
                                                Material.accent: ThemeManager.m3["tertiary"]
                                                placeholderText: "Enter Longitude"
                                                color: ThemeManager.m3["onSurface"]
                                                placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 2)
                                                selectByMouse: true
                                                selectedTextColor: ThemeManager.m3["onSurface"]
                                                selectionColor: ThemeManager.m3["inversePrimary"]
                                                background: Rectangle {
                                                    id: lonTextFieldBg
                                                    implicitWidth: latRect.width
                                                    implicitHeight: latRect.height
                                                    radius: 10
                                                    color: ThemeManager.m3["surfaceContainerLowest"]
                                                    border {
                                                        color: borderColor
                                                        width: 1
                                                    }
                                                }
                                            }
                                        }
                                        Label{
                                            id: altLabel
                                            text: qsTr("Altitude: ")
                                            color: labelColor
                                            font.pointSize: 14
                                        }
                                        Rectangle{
                                            id: altRect
                                            Layout.preferredWidth: gotoServiceColumnLayout.width / 6
                                            Layout.preferredHeight: 30
                                            color: "transparent"
                                            TextField {
                                                id: altTextField
                                                anchors {
                                                    verticalCenter: parent.verticalCenter
                                                    verticalCenterOffset: -2
                                                }
                                                leftPadding: 10
                                                topPadding: 15
                                                Material.accent: ThemeManager.m3["tertiary"]
                                                placeholderText: "Enter Altitude"
                                                color: ThemeManager.m3["onSurface"]
                                                placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 2)
                                                selectByMouse: true
                                                selectedTextColor: ThemeManager.m3["onSurface"]
                                                selectionColor: ThemeManager.m3["inversePrimary"]
                                                background: Rectangle {
                                                    id: altTextFieldBg
                                                    implicitWidth: latRect.width
                                                    implicitHeight: latRect.height
                                                    radius: 10
                                                    color: ThemeManager.m3["surfaceContainerLowest"]
                                                    border {
                                                        color: borderColor
                                                        width: 1
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    CustomTextBtn{
                                        id: gotoServiceBtn
                                        defaultColor: ThemeManager.m3["secondaryContainer"]
                                        Layout.alignment: Qt.AlignHCenter
                                        anchors.top: gpsLocationRowLayout.bottom
                                        btnLabel: "Go to Location"
                                        anchors.topMargin: 30
                                        onClicked: {
                                            gotoServiceView.goToSignal(parseFloat(latTextField.text), parseFloat(lonTextField.text), parseFloat(altTextField.text))
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
