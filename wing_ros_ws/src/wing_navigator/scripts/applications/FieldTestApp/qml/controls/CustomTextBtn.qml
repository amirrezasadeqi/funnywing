import QtQuick 2.15
import QtQuick.Controls 2.15
import "../theme" 1.0

Button{
    id: customBtn

    property color defaultColor: ThemeManager.m3["primary"]
    property color hoveredColor: Qt.darker(defaultColor, 1.3)
    property color pressedColor: Qt.darker(defaultColor, 1.6)
    property string btnLabel: qsTr("Submit")

    implicitHeight: 50
    implicitWidth: 300

    QtObject{
        id: internal

        property var dynamic_color: if(customBtn.down){
                                        pressedColor
                                    }else{
                                        customBtn.hovered ? hoveredColor : defaultColor
                                    }
    }

    background: Rectangle{
        id: btnBg
        color: internal.dynamic_color
        radius: 10
    }

    contentItem: Item {
        id: btnContent
        anchors.fill: parent
        Text {
            id: btnText
            text: customBtn.btnLabel
            color: ThemeManager.m3["onPrimary"]
            anchors.verticalCenter: parent.verticalCenter
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.bold: true
            font.pointSize: 12
            font.family: "Tahoma"
            anchors.horizontalCenter: parent.horizontalCenter
        }
    }
}
