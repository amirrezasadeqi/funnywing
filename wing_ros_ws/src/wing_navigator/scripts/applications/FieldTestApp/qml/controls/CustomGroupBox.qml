import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../theme" 1.0

Item {
    id: actionGroupBox
    implicitWidth: 1171
    implicitHeight: 171
    property string title: "actionGroupBoxTitle"
    property color bgColor: "transparent"
    property color borderColor: ThemeManager.m3["outlineVariant"]
    property int borderWidth: 3
    property var contentItem: Rectangle{
        id: contentItem
        width: 100
        height: 30
        color: ThemeManager.m3["surface"]
    }

    Rectangle {
        id: container
        color: "transparent"
        anchors.fill: parent
        GroupBox {
            id: groupBox
            title: actionGroupBox.title
            anchors.fill: parent

            background: Rectangle {
                y: groupBox.topPadding - groupBox.bottomPadding
                anchors{
                    left: parent.left
                    leftMargin: 5
                    right: parent.right
                    rightMargin: 5
                }
                height: parent.height - groupBox.topPadding + groupBox.bottomPadding
                color: actionGroupBox.bgColor
                border.color: actionGroupBox.borderColor
                border.width: actionGroupBox.borderWidth
                radius: 5
            }

            label: Label {
                x: groupBox.leftPadding
                width: groupBox.availableWidth
                text: groupBox.title
                color: ThemeManager.m3["onSurface"]
                elide: Text.ElideRight
            }

            contentItem: actionGroupBox.contentItem
        }
    }
}
