import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../controls"

Item {
    id: gotoServiceView
    implicitWidth: 1171
    implicitHeight: 171

    // To send GPS location to FieldTestApp.qml
    signal goToSignal(real lat, real lon, real alt)

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
                                            color: "#b1bbc5"
                                            font.pointSize: 14
                                        }
                                        Rectangle{
                                            id: latRect
                                            Layout.preferredWidth: gotoServiceColumnLayout.width / 6
                                            Layout.preferredHeight: 30
                                            color: "transparent"
                                            CustomTextField{
                                                id: latTextField
                                                anchors.fill: parent
                                                placeholderTextColor: "#babfba"
                                                placeholderText: "Enter Latitude"
                                                color_mouse_hover: "#033303"
                                                color_on_focus: "#005300"
                                                default_color: "#008000"
                                            }
                                        }
                                        Label{
                                            id: lonLabel
                                            text: qsTr("Longitude: ")
                                            color: "#b1bbc5"
                                            font.pointSize: 14
                                        }
                                        Rectangle{
                                            id: lonRect
                                            Layout.preferredWidth: gotoServiceColumnLayout.width / 6
                                            Layout.preferredHeight: 30
                                            color: "transparent"
                                            CustomTextField{
                                                id: lonTextField
                                                anchors.fill: parent
                                                placeholderTextColor: "#babfba"
                                                placeholderText: "Enter Longitude"
                                                color_mouse_hover: "#033303"
                                                color_on_focus: "#005300"
                                                default_color: "#008000"
                                            }
                                        }
                                        Label{
                                            id: altLabel
                                            text: qsTr("Altitude: ")
                                            color: "#b1bbc5"
                                            font.pointSize: 14
                                        }
                                        Rectangle{
                                            id: altRect
                                            Layout.preferredWidth: gotoServiceColumnLayout.width / 6
                                            Layout.preferredHeight: 30
                                            color: "transparent"
                                            CustomTextField{
                                                id: altTextField
                                                anchors.fill: parent
                                                placeholderTextColor: "#babfba"
                                                placeholderText: "Enter Altitude"
                                                color_mouse_hover: "#033303"
                                                color_on_focus: "#005300"
                                                default_color: "#008000"
                                            }
                                        }
                                    }
                                    CustomTextBtn{
                                        id: gotoServiceBtn
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
