import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: rootItem
    implicitWidth: 200
    implicitHeight: 100
    property real lat: -35.45464656
    property real lon: 51.45464656
    property real alt: 100.256
    property string latLabelText: qsTr("Lat: ")
    property string lonLabelText: qsTr("Lon: ")
    property string altLabelText: qsTr("Alt: ")

    Rectangle {
        id: bg
        color: "#2a2a2a"
        anchors.fill: parent
        clip: false

        Rectangle {
            id: container
            color: "#00ffffff"
            anchors.fill: parent
            GridLayout{
                id: gpsDisplayRowLayout
                anchors.fill: parent
                anchors.leftMargin: 5
                columns: 2
                Label{
                    id: latLabel
                    text: rootItem.latLabelText
                    color: "#b1bbc5"
                    font.pointSize: 14
                }
                Rectangle{
                    id: latRect
                    Layout.preferredWidth: 110
                    Layout.preferredHeight: 30
                    color: "#454749"
                    border.color: "green"
                    border.width: 2
                    radius: 2
                    clip: true
                    Text{
                        id: latText
                        color: "#dee3e7"
                        anchors.fill: parent
                        verticalAlignment: Text.AlignVCenter
                        anchors.leftMargin: 5
                        text: rootItem.lat
                    }
                }
                Label{
                    id: lonLabel
                    text: rootItem.lonLabelText
                    color: "#b1bbc5"
                    font.pointSize: 14
                }
                Rectangle{
                    id: lonRect
                    Layout.preferredWidth: 110
                    Layout.preferredHeight: 30
                    color: "#454749"
                    border.color: "green"
                    border.width: 2
                    radius: 2
                    clip: true
                    Text{
                        id: lonText
                        color: "#dee3e7"
                        anchors.fill: parent
                        verticalAlignment: Text.AlignVCenter
                        anchors.leftMargin: 5
                        text: rootItem.lon
                    }
                }
                Label{
                    id: altLabel
                    text: rootItem.altLabelText
                    color: "#b1bbc5"
                    font.pointSize: 14
                }
                Rectangle{
                    id: altRect
                    Layout.preferredWidth: 110
                    Layout.preferredHeight: 30
                    color: "#454749"
                    border.color: "green"
                    border.width: 2
                    radius: 2
                    clip: true
                    Text{
                        id: altText
                        color: "#dee3e7"
                        anchors.fill: parent
                        verticalAlignment: Text.AlignVCenter
                        anchors.leftMargin: 5
                        text: rootItem.alt
                    }
                }
            }
        }
    }
}
