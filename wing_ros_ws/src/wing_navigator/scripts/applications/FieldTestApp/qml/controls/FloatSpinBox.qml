import QtQuick 2.15
import QtQuick.Controls 2.15
import "../theme" 1.0

Item {
    id: root
    implicitWidth: 150
    implicitHeight: 60

    property real stepSize: 1.5
    property real from: 0
    property real to: 100
    property int numOfFlotingPoints: 4
    property real value: spinbox.value / Math.pow(10, root.numOfFlotingPoints)
    // for debug purposes to see if the intenal spin box value changes on the root value changes.
    // property int actualValue: spinbox.value

    onValueChanged: {
        spinbox.value = root.value * Math.pow(10, root.numOfFlotingPoints);
    }

    SpinBox {
        id: spinbox
        from: root.from * Math.pow(10, root.numOfFlotingPoints)
        value: 0
        to: root.to * Math.pow(10, root.numOfFlotingPoints)
        stepSize: root.stepSize * Math.pow(10, root.numOfFlotingPoints)

        anchors.fill: parent
        editable: true
        wheelEnabled: true

        contentItem: TextInput {
            z: 2
            text: spinbox.textFromValue(spinbox.value, spinbox.locale)

            font: spinbox.font
            color: ThemeManager.m3["onSurface"]
            selectionColor: ThemeManager.m3["inversePrimary"]
            selectedTextColor: ThemeManager.m3["onSurface"]
            horizontalAlignment: Qt.AlignHCenter
            verticalAlignment: Qt.AlignVCenter
            selectByMouse: true

            readOnly: !spinbox.editable
            validator: spinbox.validator
            inputMethodHints: Qt.ImhFormattedNumbersOnly
        }

        up.indicator: Rectangle {
            x: spinbox.mirrored ? 0 : parent.width - width
            height: parent.height
            implicitWidth: 40
            implicitHeight: 40
            radius: 5
            color: ThemeManager.m3["surface"]
            border.color: ThemeManager.m3["outline"]

            Text {
                text: "+"
                font.pixelSize: spinbox.font.pixelSize * 2
                color: ThemeManager.m3["onSurface"]
                anchors.fill: parent
                fontSizeMode: Text.Fit
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        down.indicator: Rectangle {
            x: spinbox.mirrored ? parent.width - width : 0
            height: parent.height
            implicitWidth: 40
            implicitHeight: 40
            radius: 5
            color: ThemeManager.m3["surface"]
            border.color: ThemeManager.m3["outline"]

            Text {
                text: "-"
                font.pixelSize: spinbox.font.pixelSize * 2
                color: ThemeManager.m3["onSurface"]
                anchors.fill: parent
                fontSizeMode: Text.Fit
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        background: Rectangle {
            implicitWidth: 140
            radius: 5
            color: ThemeManager.m3["surface"]
            border.color: ThemeManager.m3["outlineVariant"]
        }

        onValueModified: {
            root.value = spinbox.value / Math.pow(10, root.numOfFlotingPoints);
        }

        validator: DoubleValidator {
            bottom: Math.min(spinbox.from, spinbox.to)
            top:  Math.max(spinbox.from, spinbox.to)
        }

        textFromValue: function(value, locale) {
            return Number(value / Math.pow(10, root.numOfFlotingPoints)).toLocaleString(locale, 'f', root.numOfFlotingPoints)
        }

        valueFromText: function(text, locale) {
            return Number.fromLocaleString(locale, text) * Math.pow(10, root.numOfFlotingPoints)
        }
    }
}
