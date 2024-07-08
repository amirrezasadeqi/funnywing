import QtQuick 2.15
import QtQuick.Controls 2.15
import "../theme" 1.0

TextField{
    id: custom_text_field
    implicitWidth: 200
    implicitHeight: 40
    color: ThemeManager.m3["onSurface"]
    placeholderTextColor: Qt.lighter(ThemeManager.m3["onSurfaceVariant"], 3)
    placeholderText: qsTr("Type Your Name")
    antialiasing: true
    font.bold: true
    font.pointSize: 12
    font.family: "Tahoma"
    selectByMouse: true
    selectedTextColor: ThemeManager.m3["onSurface"]
    selectionColor: ThemeManager.m3["inversePrimary"]

    property color default_color: ThemeManager.m3["surfaceContainerLow"]
    property color color_mouse_hover: ThemeManager.m3["surfaceContainer"]
    property color color_on_focus: ThemeManager.m3["surfaceContainerHigh"]

    QtObject{
        id: internal

        property var dynamic_color: if(custom_text_field.focus){
                                        custom_text_field.hovered ? color_on_focus : default_color
                                    }else{
                                        custom_text_field.hoverEnabled ? color_mouse_hover : default_color
                                    }
        property var dynamicBorderColor: custom_text_field.enabled ? ThemeManager.m3["outline"] : "transparent"
    }

    background: Rectangle{
        anchors.fill: parent
        radius: 5
        color: internal.dynamic_color
        border.color: internal.dynamicBorderColor
    }
}
