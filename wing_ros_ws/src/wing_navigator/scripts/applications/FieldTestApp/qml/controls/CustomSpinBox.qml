import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtGraphicalEffects 1.15
import "../theme"

Item {
    id: root
    implicitWidth: 150
    implicitHeight: 50
    property real stepSize: 1
    signal cameraMonitorSpinBoxSignal(int zoom_percentage)

    SpinBox {
        id: spinBox
        value: 0
        from: 0
        stepSize: root.stepSize
        to: 100
        anchors.fill: parent
        editable: true
        wheelEnabled: true
        ToolTip.visible: hovered
        ToolTip.text: qsTr("Enter a zoom percentage between 0 to 100!")
        ToolTip.delay: 1000
        ToolTip.timeout: 3000

        onValueModified: {
            root.cameraMonitorSpinBoxSignal(parseInt(spinBoxTextInput.text, 10));
        }

        contentItem: TextInput {
            id: spinBoxTextInput
            z: 2
            text: spinBox.textFromValue(spinBox.value, spinBox.locale)
            horizontalAlignment: Qt.AlignHCenter
            verticalAlignment: Qt.AlignVCenter
            readOnly: !spinBox.editable
            validator: spinBox.validator
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            color: ThemeManager.m3["onSurface"]
            selectByMouse: true
        }

        up.indicator: Rectangle {
            id: upIndicator
            x: spinBox.mirrored ? 0 : parent.width - width
            implicitWidth: height
            implicitHeight: spinBox.height * 0.6
            radius: width / 2
            anchors.verticalCenter: parent.verticalCenter
            color: spinBox.up.pressed ? Qt.darker(ThemeManager.m3["tertiaryContainer"], 1.6) : ThemeManager.m3["tertiaryContainer"]


            Image {
                id: spinBoxUpImage
                source: "../../images/svg_images/zoomInIcon.svg"
                anchors.centerIn: parent
                fillMode: Image.PreserveAspectFit
                width: parent.width * 0.6
                height: width
            }

            ColorOverlay {
                anchors.fill: spinBoxUpImage
                source: spinBoxUpImage
                color: ThemeManager.m3["onPrimary"]
            }

        }

        down.indicator: Rectangle {
            id: downIndicator
            x: spinBox.mirrored ? parent.width - width : 0
            implicitWidth: height
            implicitHeight: spinBox.height * 0.6
            radius: width / 2
            anchors.verticalCenter: parent.verticalCenter
            color: spinBox.up.pressed ? Qt.darker(ThemeManager.m3["tertiaryContainer"], 1.6) : ThemeManager.m3["tertiaryContainer"]

            Image {
                id: spinBoxDownImage
                source: "../../images/svg_images/zoomOutIcon.svg"
                anchors.centerIn: parent
                fillMode: Image.PreserveAspectFit
                width: parent.width * 0.6
                height: width
            }

            ColorOverlay {
                anchors.fill: spinBoxDownImage
                source: spinBoxDownImage
                color: ThemeManager.m3["onPrimary"]
            }
        }

        background: Rectangle {
            anchors {
                left: downIndicator.horizontalCenter
                right: upIndicator.horizontalCenter
                top: upIndicator.top
                bottom: upIndicator.bottom
            }
            color: ThemeManager.m3["surfaceContainerHighest"]
            border.color: ThemeManager.m3["outline"]
        }
    }
}
