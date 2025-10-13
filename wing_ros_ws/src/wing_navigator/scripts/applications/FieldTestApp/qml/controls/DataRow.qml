import QtQuick 2.15

Item {
    id: dataRow 
    width: parent.width
    height: content.implicitHeight

    property string dataLabel: "Label"
    property string dataValue: "Value"
    property color dataColor: "white"

    Row {
        id: content
        width: parent.width
        spacing: 5
        leftPadding: 5
        rightPadding: 5

        Text {
            text: dataRow.dataLabel
            font.pixelSize: 18
            color: Qt.lighter(dataRow.dataColor, 1.5)

            width: parent.width * 0.4
            horizontalAlignment: Text.AlignLeft
        }
        
        Text {
            text: dataRow.dataValue
            font.pointSize: 12
            font.bold: true 
            color: dataRow.dataColor
            width: parent.width * 0.5
            horizontalAlignment: Text.AlignRight
        }
    }
}