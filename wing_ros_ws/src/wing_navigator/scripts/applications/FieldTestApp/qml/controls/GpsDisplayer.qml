import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import "../theme" 1.0

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
    property color labelColor: ThemeManager.m3["onSurface"]
    property color borderColor: ThemeManager.m3["outline"]
    property color valRectBg: ThemeManager.m3["surfaceBright"]
    property color valTextColor: ThemeManager.m3["onSurface"]

    Rectangle {
        id: bg
        color: ThemeManager.m3["surfaceDim"]
        anchors.fill: parent
        radius: 5
        clip: false

        Rectangle {
            id: container
            color: "transparent"
            anchors.fill: parent
            GridLayout{
                id: gpsDisplayRowLayout
                anchors.fill: parent
                anchors.leftMargin: 5
                columns: 2
                Label{
                    id: latLabel
                    text: rootItem.latLabelText
                    color: labelColor
                    font.pointSize: 14
                }
                Rectangle{
                    id: latRect
                    Layout.preferredWidth: 110
                    Layout.preferredHeight: 30
                    color: valRectBg
                    border.color: borderColor
                    border.width: 2
                    radius: 2
                    clip: true
                    Text{
                        id: latText
                        color: valTextColor
                        anchors.fill: parent
                        verticalAlignment: Text.AlignVCenter
                        anchors.leftMargin: 5
                        text: rootItem.lat
                    }
                }
                Label{
                    id: lonLabel
                    text: rootItem.lonLabelText
                    color: labelColor
                    font.pointSize: 14
                }
                Rectangle{
                    id: lonRect
                    Layout.preferredWidth: 110
                    Layout.preferredHeight: 30
                    color: valRectBg
                    border.color: borderColor
                    border.width: 2
                    radius: 2
                    clip: true
                    Text{
                        id: lonText
                        color: valTextColor
                        anchors.fill: parent
                        verticalAlignment: Text.AlignVCenter
                        anchors.leftMargin: 5
                        text: rootItem.lon
                    }
                }
                Label{
                    id: altLabel
                    text: rootItem.altLabelText
                    color: labelColor
                    font.pointSize: 14
                }
                Rectangle{
                    id: altRect
                    Layout.preferredWidth: 110
                    Layout.preferredHeight: 30
                    color: valRectBg
                    border.color: borderColor
                    border.width: 2
                    radius: 2
                    clip: true
                    Text{
                        id: altText
                        color: valTextColor
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
