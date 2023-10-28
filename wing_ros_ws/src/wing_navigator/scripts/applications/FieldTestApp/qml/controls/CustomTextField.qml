import QtQuick 2.15
import QtQuick.Controls 2.15

TextField{
    id: custom_text_field
    color: "#dddbf6"
    placeholderText: qsTr("Type Your Name")
    antialiasing: true
    font.bold: true
    font.pointSize: 12
    font.family: "Tahoma"

    property color default_color: "#2b2949"
    property color color_mouse_hover: "#1d167a"
    property color color_on_focus: "#212039"

    QtObject{
        id: internal

        property var dynamic_color: if(custom_text_field.focus){
                                        custom_text_field.hovered ? color_on_focus : default_color
                                    }else{
                                        custom_text_field.hoverEnabled ? color_mouse_hover : default_color
                                    }
    }

    background: Rectangle{
        anchors.fill: parent
        color: internal.dynamic_color
        radius: 10
    }

    selectByMouse: true
    selectedTextColor: "#1e1c38"
    selectionColor: "#7e76ee"

}
