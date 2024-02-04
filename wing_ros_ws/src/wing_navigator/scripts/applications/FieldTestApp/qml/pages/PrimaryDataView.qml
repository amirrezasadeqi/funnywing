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
    property real distToTg: 50.46
    property real tgRecvDataRate: 5.0
    property real tgRelAlt: 50.0

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

                Flickable {
                    id: wingDataFlickable
                    anchors.fill: parent
                    contentHeight: wingDataGridLayout.height
                    contentWidth: wingDataGridLayout.width
                    boundsBehavior: Flickable.DragAndOvershootBounds
                    ScrollBar.horizontal: ScrollBar{
                        id: wingDataHorizontalScrollBar
                    }
                    ScrollBar.vertical: ScrollBar{
                        id: wingDataVerticalScrollBar
                    }

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
                            MouseArea{
                                id: wingGpsLabelMouseArea
                                anchors.fill: parent
                                onClicked: {
                                    wingGpsContainer.Layout.preferredHeight = wingGpsContainer.height != 0 ? 0 : 180;
                                }
                            }
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
                            MouseArea{
                                id: wingVelocityLabelMouseArea
                                anchors.fill: parent
                                onClicked: {
                                    wingVelocityContainer.Layout.preferredHeight = wingVelocityContainer.height != 0 ? 0 : 180;
                                }
                            }
                        }
                        Rectangle{
                            id: wingVelocityContainer
                            Layout.preferredWidth: 180
                            Layout.preferredHeight: 0
                            color: "transparent"
                            anchors.top: wingVelocityLabel.top
                            clip: true
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

                Flickable {
                    id: targetDataFlickable
                    anchors.fill: parent
                    contentHeight: targetDataGridLayout.height
                    contentWidth: targetDataGridLayout.width
                    boundsBehavior: Flickable.DragAndOvershootBounds
                    ScrollBar.horizontal: ScrollBar{
                        id: targetDataHorizontalScrollBar
                    }
                    ScrollBar.vertical: ScrollBar{
                        id: targetDataVerticalScrollBar
                    }

                    GridLayout {
                        id: targetDataGridLayout
                        x: 0
                        y: 0
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
                        Label{
                            id: distToTgLabel
                            color: "#d1dde9"
                            text: qsTr("Distance to Target")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: distToTgRect
                            Layout.preferredWidth: 110
                            Layout.preferredHeight: 30
                            anchors.top: distToTgLabel.top
                            color: "#454749"
                            border.color: "green"
                            border.width: 2
                            radius: 2
                            clip: true
                            Text{
                                id: distToTgText
                                color: "#dee3e7"
                                anchors.fill: parent
                                verticalAlignment: Text.AlignVCenter
                                anchors.leftMargin: 5
                                text: rootItem.distToTg
                            }
                        }
                        Label{
                            id: tgDataRateLabel
                            color: "#d1dde9"
                            text: qsTr("Target Recieved Data Rate")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: tgDataRateRect
                            Layout.preferredWidth: 110
                            Layout.preferredHeight: 30
                            anchors.top: tgDataRateLabel.top
                            color: "#454749"
                            border.color: "green"
                            border.width: 2
                            radius: 2
                            clip: true
                            Text{
                                id: tgDataRateText
                                color: "#dee3e7"
                                anchors.fill: parent
                                verticalAlignment: Text.AlignVCenter
                                anchors.leftMargin: 5
                                text: rootItem.tgRecvDataRate.toFixed(2)
                            }
                        }
                        Label{
                            id: tgRelAltLabel
                            color: "#d1dde9"
                            text: qsTr("Target Relative Altitude")
                            Layout.topMargin: 20
                            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                        }
                        Rectangle{
                            id: tgRelAltRect
                            Layout.preferredWidth: 110
                            Layout.preferredHeight: 30
                            anchors.top: tgRelAltLabel.top
                            color: "#454749"
                            border.color: "green"
                            border.width: 2
                            radius: 2
                            clip: true
                            Text{
                                id: tgRelAltText
                                color: "#dee3e7"
                                anchors.fill: parent
                                verticalAlignment: Text.AlignVCenter
                                anchors.leftMargin: 5
                                text: rootItem.tgRelAlt
                            }
                        }
                    }
                }
            }
        }
    }
}
