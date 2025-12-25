import QtQuick 2.12
import QtQuick.Controls 2.12
import "../theme" 1.0

Item {
    id: root
    implicitWidth: 400
    implicitHeight: 100
    property var modelData: ["MANUAL", "STABILIZE", "AUTO", "GUIDED", "RTL", "FBWA"]
    property color defautlBgColor: ThemeManager.m3["surfaceContainer"]
    property color clickedBgColor: ThemeManager.m3["surfaceContainerHighest"]
    property color defaultBorderColor: ThemeManager.m3["outlineVariant"]
    property color clickedBorderColor: ThemeManager.m3["outline"]
    property string currentText: customComboBox.currentText
    property alias font: customComboBox.font

    ComboBox {
        id: customComboBox
        anchors.fill: parent

        model: root.modelData
        wheelEnabled: true
        editable: true

        delegate: ItemDelegate {
            width: cbPopup.width
            leftPadding: 8
            rightPadding: 8
            contentItem: Text {
                width: parent.width
                text: modelData
                color: ThemeManager.m3["onSurface"]
                font.pointSize: 15
                elide: Text.ElideRight
                wrapMode: Text.NoWrap
                verticalAlignment: Text.AlignVCenter
            }
            highlighted: customComboBox.highlightedIndex === index
        }

        indicator: Canvas {
            id: canvas
            x: customComboBox.width - width - customComboBox.rightPadding
            y: customComboBox.topPadding + (customComboBox.availableHeight - height) / 2
            width: 12
            height: 8
            contextType: "2d"

            Connections {
                target: customComboBox
                function onPressedChanged() { canvas.requestPaint(); }
            }

            onPaint: {
                context.reset();
                context.moveTo(0, 0);
                context.lineTo(width, 0);
                context.lineTo(width / 2, height);
                context.closePath();
                context.fillStyle = customComboBox.pressed ? ThemeManager.m3["tertiary"] : ThemeManager.m3["tertiaryContainer"];
                context.fill();
            }
        }

        contentItem: Text {
            id: displayedText
            leftPadding: 5
            rightPadding: customComboBox.indicator.width + customComboBox.spacing
            text: customComboBox.displayText
            font: customComboBox.font
            color: customComboBox.pressed ? ThemeManager.m3["onSurface"] : ThemeManager.m3["onSurfaceVariant"]
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
        }

        background: Rectangle {
            implicitWidth: 400
            implicitHeight: 40
            border.color: customComboBox.pressed ? clickedBorderColor : defaultBorderColor
            color: customComboBox.pressed ? clickedBgColor : defautlBgColor
            border.width: customComboBox.visualFocus ? 2 : 1
            radius: 2
        }

        popup: Popup {
            id: cbPopup
            parent: customComboBox
            x: 0
            y: customComboBox.height - 1
            width: customComboBox.width
            padding: 0
            implicitHeight: Math.min(listView.contentHeight + 2, 200)

            contentItem: ListView {
                id: listView
                anchors.fill: parent
                anchors.margins: 1
                clip: true
                model: customComboBox.popup.visible ? customComboBox.delegateModel : null
                currentIndex: customComboBox.highlightedIndex
                boundsBehavior: Flickable.StopAtBounds
                ScrollIndicator.vertical: ScrollIndicator { active: true }
            }

            background: Rectangle {
                anchors.fill: parent
                border.color: clickedBorderColor
                color: clickedBgColor
                radius: 2
            }
        }
    }
}
