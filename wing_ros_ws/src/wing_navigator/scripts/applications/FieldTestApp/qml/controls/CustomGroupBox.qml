import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: actionGroupBox
    implicitWidth: 1171
    implicitHeight: 171
    property string title: "actionGroupBoxTitle"
    property color bgColor: "transparent"
    property color borderColor: "green"
    property int borderWidth: 3
    property var contentItem: Rectangle{
        id: contentItem
        width: 100
        height: 30
        color: "green"
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
                color: "#21be2b"
                elide: Text.ElideRight
            }

            contentItem: actionGroupBox.contentItem
        }
    }
}
