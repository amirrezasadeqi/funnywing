import QtQuick 2.15
import QtQuick.Controls 2.15

Button{
    id: customBtn

    property color defaultColor: "#6bcad7"
    property color hoveredColor: "#21c5db"
    property color pressedColor: "#054952"
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
