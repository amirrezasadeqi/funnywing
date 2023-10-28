import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../controls"

Item {
    id: rootItem
    implicitWidth: 527
    implicitHeight: 307
    property real heading: 270.36
    property string wingFlightState: qsTr("MANUAL")
    property real wingRelativeAlt: 100.45455
    property real wingGpsLatVal: -35.56565555
    property real wingGpsLonVal: 51.454554545
    property real wingGpsAltVal: 100.00
    property real wingVelValX: -11.1111
    property real wingVelValY: 2.545444
    property real wingVelValZ: 0.4465465
    property real tgGpsLatVal: -35.56565555
    property real tgGpsLonVal: 51.454554545
    property real tgGpsAltVal: 100.00

    Rectangle{
        id: bg
        color: "#e8232323"
        anchors.fill: parent

        Rectangle {
            id: container
            color: "#00ffffff"
            anchors.fill: parent

            Rectangle {
                id: wingDataContainer
                width: parent.width / 2
                color: "#00ffffff"
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                clip: true

                ScrollView {
                    id: wingDataScrollView
                    anchors.fill: parent
                    GridLayout {
                        id: wingDataGridLayout
                        columnSpacing: 0
                        rowSpacing: 0
                        columns: 2
                        Label{
                            id: wingGpsLabel
                            color: "#d1dde9"
                            text: qsTr("Wing GPS")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: wingGpsContainer
                            Layout.preferredWidth: 180
                            Layout.preferredHeight: 180
                            color: "transparent"
                            anchors.top: wingGpsLabel.top
                            clip: true
                            GpsDisplayer{
                                id: wingGpsDisplayer
                                anchors.fill: parent
                                anchors.topMargin: -15
                                lat: rootItem.wingGpsLatVal
                                lon: rootItem.wingGpsLonVal
                                alt: rootItem.wingGpsAltVal
                            }
                        }
                        Label{
                            id: wingVelocityLabel
                            color: "#d1dde9"
                            text: qsTr("Wing Velocity")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: wingVelocityContainer
                            Layout.preferredWidth: 180
                            Layout.preferredHeight: 180
                            color: "transparent"
                            anchors.top: wingVelocityLabel.top
                            clip: false
                            GpsDisplayer{
                                id: wingVelocityDisplayer
                                anchors.fill: parent
                                anchors.topMargin: -15
                                latLabelText: qsTr("Vx: ")
                                lonLabelText: qsTr("Vy: ")
                                altLabelText: qsTr("Vz: ")
                                lat: rootItem.wingVelValX
                                lon: rootItem.wingVelValY
                                alt: rootItem.wingVelValZ
                            }
                        }
                        Label{
                            id: wingHeadingLabel
                            color: "#d1dde9"
                            text: qsTr("Wing Heading")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: headingRect
                            Layout.preferredWidth: 110
                            Layout.preferredHeight: 30
                            anchors.top: wingHeadingLabel.top
                            color: "#454749"
                            border.color: "green"
                            border.width: 2
                            radius: 2
                            clip: true
                            Text{
                                id: headingText
                                color: "#dee3e7"
                                anchors.fill: parent
                                verticalAlignment: Text.AlignVCenter
                                anchors.leftMargin: 5
                                text: rootItem.heading
                            }
                        }
                        Label{
                            id: wingFlightStateLabel
                            color: "#d1dde9"
                            text: qsTr("Wing Flight State")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: wingFlightStateRect
                            Layout.preferredWidth: 110
                            Layout.preferredHeight: 30
                            anchors.top: wingFlightStateLabel.top
                            color: "#454749"
                            border.color: "green"
                            border.width: 2
                            radius: 2
                            clip: true
                            Text{
                                id: wingFlightStateText
                                color: "#dee3e7"
                                anchors.fill: parent
                                verticalAlignment: Text.AlignVCenter
                                anchors.leftMargin: 5
                                text: rootItem.wingFlightState
                            }
                        }
                        Label{
                            id: wingRelativeAltLabel
                            color: "#d1dde9"
                            text: qsTr("Wing Relative Altitude")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: wingRelativeAltContainer
                            Layout.preferredWidth: 110
                            Layout.preferredHeight: 30
                            anchors.top: wingRelativeAltLabel.top
                            Layout.bottomMargin: height
                            color: "#454749"
                            border.color: "green"
                            border.width: 2
                            radius: 2
                            clip: false
                            Text{
                                id: wingRelativeAltText
                                color: "#dee3e7"
                                anchors.fill: parent
                                verticalAlignment: Text.AlignVCenter
                                anchors.leftMargin: 5
                                text: rootItem.wingRelativeAlt
                            }
                        }
                    }
                }
            }

            Rectangle {
                id: targetDataContainer
                x: 320
                y: 0
                color: "#00ffffff"
                anchors.left: wingDataContainer.right
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                clip: true
                anchors.leftMargin: 0

                GridLayout {
                    id: targetDataGridLayout
                    anchors.fill: parent
                    columnSpacing: 2
                    columns: 2
                    Label{
                        id: tgGpsLabel
                        color: "#d1dde9"
                        text: qsTr("Traget GPS")
                        Layout.topMargin: 20
                        Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                    }
                    Rectangle{
                        id: tgGpsContainer
                        Layout.preferredWidth: 180
                        Layout.preferredHeight: 180
                        color: "transparent"
                        anchors.top: tgGpsLabel.top
                        clip: true
                        GpsDisplayer{
                            id: tgGpsDisplayer
                            anchors.fill: parent
                            anchors.topMargin: -15
                            lat: rootItem.tgGpsLatVal
                            lon: rootItem.tgGpsLonVal
                            alt: rootItem.tgGpsAltVal
                        }
                    }
                }
            }
        }
    }
}
