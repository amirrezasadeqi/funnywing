import QtQuick 2.15
import QtQuick.Controls 2.15
import "../controls"
import "../theme" 1.0

Item {
    id: root
    implicitWidth: 300
    implicitHeight: 60

    property string labelText: "P"
    property alias value: spinBox.value
    property alias stepSize: stepSizeField.text
    property alias from: spinBox.from
    property alias to: spinBox.to
    property alias numOfFlotingPoints: spinBox.numOfFlotingPoints

    Row {
        spacing: 25
        anchors.fill: parent
        Row {
            spacing: 3
            width: 223
            height: parent.height

            Label {
                text: labelText
                width: 25
                font.pixelSize: 18
                color: ThemeManager.m3["onSurface"]
                horizontalAlignment: Text.AlignHCenter
                anchors.verticalCenter: parent.verticalCenter
            }

            FloatSpinBox {
                id: spinBox
                // To load from saved files, set the value of this object:
                // spinBox.value = loadedValue
                width: 180
                from: 0
                to: 100
                numOfFlotingPoints: 5
                stepSize: parseFloat(stepSizeField.text) || 0.1
            }
        }

        CustomTextField {
            id: stepSizeField
            width: 85
            text: "0.1"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            leftPadding: 5
            rightPadding: 5
            bottomPadding: 2
            topPadding: 2
            anchors.verticalCenter: parent.verticalCenter
            placeholderText: "Step"
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            maximumLength: root.numOfFlotingPoints + 3
            validator: DoubleValidator {}
            onEditingFinished: {
                var step = parseFloat(text)
                if (isNaN(step) || step <= 0) {
                    step = 0.1 // Default step if invalid
                    text = step.toString()
                }
                spinBox.stepSize = step
            }
        }
    }
}