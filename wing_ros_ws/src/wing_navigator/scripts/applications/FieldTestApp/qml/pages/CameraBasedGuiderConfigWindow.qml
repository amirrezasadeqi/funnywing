import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Window 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import Qt.labs.platform 1.1
import Qt.labs.settings 1.1
import "../controls"
import "../theme" 1.0

Window {
    id: configWindow
    width: 900
    height: 850
    title: "Camera Based Guidance Configuration"
    color: ThemeManager.m3["surface"]
    modality: Qt.NonModal
    visible: false
    flags: Qt.Window | Qt.WindowTitleHint | Qt.WindowCloseButtonHint | Qt.CustomizeWindowHint

    property var pidControllers: [[], []]
    property var sigmoidCoefficients: []

    signal cameraBasedGuiderApplyConfigsSignal(var configs)

    function safeParseFloat(text, defVal) {
        var v = parseFloat(text)
        return isNaN(v) ? defVal : v
    }

    function clearAllFields() {
        constThrottleTextField.text = ""
        xpidSaturationLowTextField.text = ""
        xpidSaturationHighTextField.text = ""
        ypidSaturationLowTextField.text = ""
        ypidSaturationHighTextField.text = ""

        for (let i = 0; i < pidControllers[0].length; i++) {
            if (pidControllers[0][i].value !== undefined) pidControllers[0][i].value = ""
        }
        for (let i = 0; i < pidControllers[1].length; i++) {
            if (pidControllers[1][i].value !== undefined) pidControllers[1][i].value = ""
        }

        for (let i = 0; i < sigmoidCoefficients.length; i++) {
            sigmoidCoefficients[i].coefficientTextValue = ""
        }
    }

    Component.onCompleted: {
        clearAllFields()
    }

    Rectangle {
        id: formContainer
        width: parent.width * 0.98
        radius: 10
        border.color: ThemeManager.m3["outline"]
        color: ThemeManager.m3["surfaceContainerHigh"]

        anchors.top: parent.top
        anchors.topMargin: parent.height * 0.03
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: buttonBar.top
        anchors.bottomMargin: 10

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 15
            spacing: 15

            RowLayout {
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                spacing: 10

                RowLayout {
                    Layout.preferredWidth: 100
                    Layout.fillHeight: true
                    spacing: 4

                    Label {
                        id: profileTypeLabel
                        color: ThemeManager.m3["onSurface"]
                        text: profileSwitch.checked ? qsTr("Sigmoid Profile") : qsTr("Constant Profile")
                        font.pixelSize: 18
                        Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                        Layout.preferredWidth: 150
                    }
                    Switch {
                        id: profileSwitch
                        Material.accent: ThemeManager.m3["tertiaryContainer"]
                        Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    spacing: 8
                    Layout.leftMargin: 235

                    Label {
                        id: constThrottleLabel
                        text: qsTr("Constant Throttle")
                        font.pixelSize: 18
                        color: ThemeManager.m3["onSurface"]
                        Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                        Layout.leftMargin: 20
                    }
                    CustomTextField {
                        id: constThrottleTextField
                        Layout.preferredWidth: 80
                        leftPadding: 5
                        rightPadding: 5
                        bottomPadding: 2
                        topPadding: 2
                        Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                        placeholderText: ""
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        maximumLength: 4
                        validator: DoubleValidator { bottom: 0.0; top: 1.0 }
                        HoverHandler { id: constThrottleHoverHandler }
                        ToolTip.visible: constThrottleHoverHandler.hovered
                        ToolTip.text: qsTr("Enter throttle value, [0, 1.0], for the constant profile.")
                        ToolTip.delay: 1000
                        ToolTip.timeout: 3000
                        horizontalAlignment: TextInput.AlignHCenter
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 8

                GroupBox {
                    id: xpidGroupBox
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 400
                    Layout.alignment: Qt.AlignTop

                    background: Rectangle {
                        id: xpidBackground
                        color: ThemeManager.m3["surfaceContainerHigh"]
                        border.color: ThemeManager.m3["outlineVariant"]
                        border.width: 1

                        layer.effect: BorderImage {
                            border { top: 1; left: 1; right: 1; bottom: 0}
                        }
                    }

                    label: Item {
                        id: xpidHeader
                        width: parent.width
                        height: 50

                        Rectangle {
                            anchors.fill: parent
                            color: "#ccbbbbbb" 
                        }

                        Rectangle {
                            height: 1
                            width: parent.width
                            color: ThemeManager.m3["outlineVariant"]
                            anchors.top: parent.top
                            anchors.left: parent.left
                            anchors.right: parent.right
                        }

                        Rectangle {
                            width: 1
                            height: parent.height
                            color: ThemeManager.m3["outlineVariant"]
                            anchors.left: parent.left
                        }

                        Rectangle {
                            width: 1
                            height: parent.height
                            color: ThemeManager.m3["outlineVariant"]
                            anchors.right: parent.right
                        }

                        Label {
                            text: qsTr("X PIDs")
                            color: ThemeManager.m3["onSurface"]
                            anchors.centerIn: parent
                            font.pixelSize: 24
                            font.bold: true
                        }
                    }

                    contentItem: ColumnLayout {
                        anchors.top: parent.top
                        anchors.topMargin: xpidHeader.height
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        anchors.leftMargin: 10
                        anchors.rightMargin: 10
                        anchors.bottomMargin: 10 
                        spacing: 12

                        RowLayout {
                            id: rowLayout
                            anchors.left: parent.left
                            anchors.leftMargin: 0
                            spacing: 8
                            CheckDelegate {
                                id: xpidDisableCheckBox
                                text: qsTr("Disable")
                                font.pixelSize: 18
                                Material.accent: ThemeManager.m3["onSurface"]
                                Material.foreground: ThemeManager.m3["onSurface"]
                            }
                            Item {
                                Layout.preferredWidth: 69
                            }
                            CheckDelegate {
                                id: xpidLockCheckBox
                                text: qsTr("Lock")
                                font.pixelSize: 18
                                Material.accent: ThemeManager.m3["onSurface"]
                                Material.foreground: ThemeManager.m3["onSurface"]
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            spacing: 8

                            Rectangle {
                                id: xpidSaturationLowContainer
                                Layout.fillWidth: true
                                Layout.preferredHeight: 40
                                color: "#ccbbbbbb"
                                radius: 5

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.leftMargin: 12
                                    spacing: 8

                                    Label {
                                        id: xpidSaturationLowLabel
                                        text: qsTr("saturation low")
                                        color: ThemeManager.m3["onSurface"]
                                        font.pixelSize: 18
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                                    }
                                    CustomTextField {
                                        id: xpidSaturationLowTextField
                                        topPadding: 5
                                        bottomPadding: 5 
                                        Layout.preferredWidth: 55
                                        font.pointSize: 12
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                        horizontalAlignment: TextInput.AlignHCenter
                                        verticalAlignment: TextInput.AlignVCenter
                                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                                        validator: DoubleValidator {}
                                        placeholderText: ""
                                    }
                                }
                            }

                            Rectangle {
                                id: xpidSaturationHighContainer
                                Layout.fillWidth: true
                                Layout.preferredHeight: 40
                                color: "#ccbbbbbb"
                                radius: 5

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.leftMargin: 12
                                    spacing: 8

                                    Label {
                                        id: xpidSaturationHighLabel
                                        text: qsTr("saturation High")
                                        color: ThemeManager.m3["onSurface"]
                                        font.pixelSize: 18
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                                    }
                                    CustomTextField {
                                        id: xpidSaturationHighTextField
                                        topPadding: 5
                                        bottomPadding: 5 
                                        Layout.preferredWidth: 50
                                        font.pointSize: 12
                                        horizontalAlignment: TextInput.AlignHCenter
                                        verticalAlignment: TextInput.AlignVCenter
                                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                        validator: DoubleValidator {}
                                        placeholderText: ""
                                    }
                                }
                            }
                        }

                        ColumnLayout {
                            id: xpidContainer
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            spacing: 10

                            Layout.leftMargin: 0

                            Repeater {
                                model: [ {"labelText": "P"}, {"labelText": "I"}, {"labelText": "D"} ]
                                delegate: PIDConfigRow {
                                    Layout.fillWidth: false
                                    labelText: modelData.labelText
                                    enabled: !xpidLockCheckBox.checked

                                    Layout.preferredWidth: xpidGroupBox.width * 0.7
                                }
                            }

                            Component.onCompleted: {
                                configWindow.pidControllers[0] = []
                                for (let i = 0; i < xpidContainer.children.length; i++) {
                                    let c = xpidContainer.children[i]
                                    if (c && c.labelText !== undefined) configWindow.pidControllers[0].push(c)
                                }
                            }
                        }
                    }
                }

                GroupBox {
                    id: ypidGroupBox
                    title: qsTr("Y PIDs")
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 400
                    Layout.alignment: Qt.AlignTop

                    background: Rectangle {
                        id: ypidBackground
                        color: ThemeManager.m3["surfaceContainerHigh"]
                        border.color: ThemeManager.m3["outlineVariant"]
                        border.width: 1

                        layer.effect: BorderImage {
                            border { top: 1; left: 1; right: 1; bottom: 0}
                        }
                    }

                    label: Item {
                        id: ypidHeader
                        width: parent.width
                        height: 50

                        Rectangle {
                            anchors.fill: parent
                            color: "#ccbbbbbb"
                        }

                        Rectangle {
                            height: 1
                            width: parent.width
                            color: ThemeManager.m3["outlineVariant"]
                            anchors.top: parent.top
                            anchors.left: parent.left
                            anchors.right: parent.right
                        }

                        Rectangle {
                            width: 1
                            height: parent.height
                            color: ThemeManager.m3["outlineVariant"]
                            anchors.left: parent.left
                        }

                        Rectangle {
                            width: 1
                            height: parent.height
                            color: ThemeManager.m3["outlineVariant"]
                            anchors.right: parent.right
                        }

                        Label {
                            text: qsTr("Y PIDs")
                            color: ThemeManager.m3["onSurface"]
                            anchors.centerIn: parent
                            font.pixelSize: 24
                            font.bold: true
                        }
                    }

                    contentItem: ColumnLayout {
                        anchors.top: parent.top
                        anchors.topMargin: ypidHeader.height
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        anchors.leftMargin: 10
                        anchors.rightMargin: 10
                        anchors.bottomMargin: 10 
                        spacing: 12

                        RowLayout {
                            anchors.left: parent.left
                            anchors.leftMargin: 0
                            spacing: 8
                            CheckDelegate {
                                id: ypidDisableCheckBox
                                text: qsTr("Disable")
                                font.pixelSize: 18
                                Material.accent: ThemeManager.m3["tertiaryContainer"]
                                Material.foreground: ThemeManager.m3["onSurface"]
                            }
                            Item {
                                Layout.preferredWidth: 69
                            }
                            CheckDelegate {
                                id: ypidLockCheckBox
                                text: qsTr("Lock")
                                font.pixelSize: 18
                                Material.accent: ThemeManager.m3["tertiaryContainer"]
                                Material.foreground: ThemeManager.m3["onSurface"]
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            spacing: 8

                            Rectangle {
                                id: ypidSaturationLowContainer
                                Layout.fillWidth: true
                                Layout.preferredHeight: 40
                                color: "#ccbbbbbb"
                                radius: 5

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.leftMargin: 12
                                    spacing: 8
                                    Label {
                                        id: ypidSaturationLowLabel
                                        text: qsTr("saturation low")
                                        color: ThemeManager.m3["onSurface"]
                                        font.pixelSize: 18
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                                    }
                                    CustomTextField {
                                        id: ypidSaturationLowTextField
                                        topPadding: 5
                                        bottomPadding: 5 
                                        Layout.preferredWidth: 55
                                        font.pointSize: 12
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                        horizontalAlignment: TextInput.AlignHCenter
                                        verticalAlignment: TextInput.AlignVCenter
                                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                                        validator: DoubleValidator {}
                                        placeholderText: ""
                                    }
                                }
                            }

                            Rectangle {
                                id: ypidSaturationHighContainer
                                Layout.fillWidth: true
                                Layout.preferredHeight: 40
                                color: "#ccbbbbbb"
                                radius: 5

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.leftMargin: 12
                                    spacing: 8
                                    Label {
                                        id: ypidSaturationHighLabel
                                        text: qsTr("saturation High")
                                        color: ThemeManager.m3["onSurface"]
                                        font.pixelSize: 18
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                                    }
                                    CustomTextField {
                                        id: ypidSaturationHighTextField
                                        topPadding: 5
                                        bottomPadding: 5 
                                        Layout.preferredWidth: 50
                                        font.pointSize: 12
                                        horizontalAlignment: TextInput.AlignHCenter
                                        verticalAlignment: TextInput.AlignVCenter
                                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                                        Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                        validator: DoubleValidator {}
                                        placeholderText: ""
                                    }
                                }
                            }
                        }

                        ColumnLayout {
                            id: ypidContainer
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            spacing: 10

                            Repeater {
                                model: [ {"labelText": "P"}, {"labelText": "I"}, {"labelText": "D"} ]
                                delegate: PIDConfigRow {
                                    Layout.fillWidth: true
                                    labelText: modelData.labelText
                                    enabled: !ypidLockCheckBox.checked
                                }
                            }

                            Component.onCompleted: {
                                configWindow.pidControllers[1] = []
                                for (let i = 0; i < ypidContainer.children.length; i++) {
                                    let c = ypidContainer.children[i]
                                    if (c && c.labelText !== undefined) configWindow.pidControllers[1].push(c)
                                }
                            }
                        }
                    }
                }
            }

            GroupBox {
                id: sigmoidProfileConstsGroupBox
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumHeight: 240

                background: Rectangle {
                    id: sigmoidBackground
                    color: ThemeManager.m3["surfaceContainerHigh"]
                    border.color: ThemeManager.m3["outlineVariant"]
                    border.width: 1

                    layer.effect: BorderImage {
                        border { top: 1; left: 1; right: 1; bottom: 0}
                    }
                }

                label: Item {
                    id: sigmoidHeader
                    width: parent.width
                    height: 50

                    Rectangle {
                        anchors.fill: parent
                        color: "#ccbbbbbb"
                    }

                    Rectangle {
                        height: 1
                        width: parent.width
                        color: ThemeManager.m3["outlineVariant"]
                        anchors.top: parent.top
                        anchors.left: parent.left
                        anchors.right: parent.right
                    }

                    Rectangle {
                        width: 1
                        height: parent.height
                        color: ThemeManager.m3["outlineVariant"]
                        anchors.left: parent.left
                    }

                    Rectangle {
                        width: 1
                        height: parent.height
                        color: ThemeManager.m3["outlineVariant"]
                        anchors.right: parent.right
                    }

                    Label {
                        text: qsTr("Sigmoid Profile Constants")
                        color: ThemeManager.m3["onSurface"]
                        anchors.centerIn: parent
                        font.pixelSize: 24
                        font.bold: true
                    }
                }

                GridLayout {
                    id: sigmoidProfileConstsGrid
                    columns: 2
                    columnSpacing: 10
                    rowSpacing: 10
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.topMargin: 50

                    anchors.leftMargin: 10
                    anchors.rightMargin: 10
                    anchors.bottomMargin: 30
                    Repeater {
                        id: sigmoidRepeater
                        model: [
                            {"labelName": "a"},
                            {"labelName": "b"},
                            {"labelName": "size_threshold"},
                            {"labelName": "wing_too_below_threshold"},
                            {"labelName": "wing_too_below_throttle"},
                            {"labelName": "wing_too_above_threshold"},
                            {"labelName": "wing_too_above_throttle"},
                            {"labelName": "wing_tg_at_same_level_throttle"}
                        ]
                        delegate: Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            color: ThemeManager.m3["surfaceDim"]
                            border.color: ThemeManager.m3["outlineVariant"]
                            radius: 5

                            property string coefficientName: coefficientLabel.text
                            property alias coefficientTextValue: coefficientTextField.text

                            RowLayout {
                                anchors.fill: parent
                                spacing: 8

                                Label {
                                    id: coefficientLabel
                                    text: modelData.labelName
                                    font.pixelSize: 18
                                    color: ThemeManager.m3["onSurface"]
                                    Layout.leftMargin: 5
                                    Layout.preferredWidth: 250
                                    Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
                                }

                                CustomTextField {
                                    id: coefficientTextField
                                    Layout.preferredWidth: 100
                                    topPadding: 5
                                    bottomPadding: 5
                                    Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                    inputMethodHints: Qt.ImhFormattedNumbersOnly
                                    validator: DoubleValidator {}
                                    placeholderText: ""
                                    horizontalAlignment: TextInput.AlignHCenter
                                    verticalAlignment: TextInput.AlignVCenter
                                }
                            }
                        }
                    }

                    Component.onCompleted: {
                        configWindow.sigmoidCoefficients = []
                        for (let i = 0; i < sigmoidRepeater.count; i++) {
                            let it = sigmoidRepeater.itemAt(i)
                            if (it !== null) configWindow.sigmoidCoefficients.push(it)
                        }
                    }
                }
            }
        }
    }

    Rectangle {
        id: buttonBar
        width: formContainer.width
        height: 60
        color: ThemeManager.m3["surfaceContainer"]
        radius: 5
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        anchors.horizontalCenter: parent.horizontalCenter

        RowLayout {
            id: controlContainerRow
            anchors.horizontalCenter: parent.horizontalCenter 
            anchors.verticalCenter: parent.verticalCenter
            spacing: 15

            CustomTextBtn {
                id: applyBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Apply")
                Layout.preferredWidth: buttonBar.width * 0.2
                onClicked: {
                    let profileType = (profileSwitch.checked) ? 1 : 0;
                    let constThrottle = safeParseFloat(constThrottleTextField.text, 0.0);
                    let xpids = [
                        configWindow.pidControllers[0][0].value,
                        configWindow.pidControllers[0][1].value,
                        configWindow.pidControllers[0][2].value,
                    ];
                    let xpidSaturations = [
                        safeParseFloat(xpidSaturationLowTextField.text, 0.0),
                        safeParseFloat(xpidSaturationHighTextField.text, 0.0)
                    ];
                    let ypids = [
                        configWindow.pidControllers[1][0].value,
                        configWindow.pidControllers[1][1].value,
                        configWindow.pidControllers[1][2].value,
                    ];
                    let ypidSaturations = [
                        safeParseFloat(ypidSaturationLowTextField.text, 0.0),
                        safeParseFloat(ypidSaturationHighTextField.text, 0.0)
                    ];
                    let sigmoidCoeff = [
                        safeParseFloat(configWindow.sigmoidCoefficients[0].coefficientTextValue, 0.0),
                        safeParseFloat(configWindow.sigmoidCoefficients[1].coefficientTextValue, 0.0),
                        safeParseFloat(configWindow.sigmoidCoefficients[2].coefficientTextValue, 0.0),
                        safeParseFloat(configWindow.sigmoidCoefficients[3].coefficientTextValue, 0.0),
                        safeParseFloat(configWindow.sigmoidCoefficients[4].coefficientTextValue, 0.0),
                        safeParseFloat(configWindow.sigmoidCoefficients[5].coefficientTextValue, 0.0),
                        safeParseFloat(configWindow.sigmoidCoefficients[6].coefficientTextValue, 0.0),
                        safeParseFloat(configWindow.sigmoidCoefficients[7].coefficientTextValue, 0.0)
                    ];
                    var configs = {
                        "profile_type": profileType,
                        "const_throttle": constThrottle,
                        "x_pids": (!xpidDisableCheckBox.checked) ? xpids : [0, 0, 0],
                        "xpid_saturations": xpidSaturations,
                        "y_pids": (!ypidDisableCheckBox.checked) ? ypids : [0, 0, 0],
                        "ypid_saturations": ypidSaturations,
                        "a": sigmoidCoeff[0],
                        "b": sigmoidCoeff[1],
                        "size_threshold": sigmoidCoeff[2],
                        "wing_too_below_threshold": sigmoidCoeff[3],
                        "wing_too_below_throttle": sigmoidCoeff[4],
                        "wing_too_above_threshold": sigmoidCoeff[5],
                        "wing_too_above_throttle": sigmoidCoeff[6],
                        "wing_tg_at_same_level_throttle": sigmoidCoeff[7]
                    };
                    configWindow.cameraBasedGuiderApplyConfigsSignal(configs);
                }
            }

            CustomTextBtn {
                id: saveBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Save")
                Layout.preferredWidth: buttonBar.width * 0.2
                onClicked: { saveDialog.open() }
            }

            CustomTextBtn {
                id: loadBtn
                defaultColor: ThemeManager.m3["secondaryContainer"]
                btnLabel: qsTr("Load")
                Layout.preferredWidth: buttonBar.width * 0.2
                onClicked: { loadDialog.open() }
            }
        }
    }

    Settings {
        id: configSettings
        fileName: StandardPaths.writableLocation(StandardPaths.DocumentsLocation) + "/config.ini"
        property int profile_type
        property real const_throttle
        property var x_pids
        property var y_pids
        property var xpid_saturations
        property var ypid_saturations
        property real a
        property real b
        property real size_threshold
        property real wing_too_below_threshold
        property real wing_too_below_throttle
        property real wing_too_above_threshold
        property real wing_too_above_throttle
        property real wing_tg_at_same_level_throttle
        property bool x_disable
        property bool x_lock
        property bool y_disable
        property bool y_lock
    }

    FileDialog {
        id: saveDialog
        title: "Save Config As"
        fileMode: FileDialog.SaveFile
        nameFilters: ["Config files (*.ini)", "All files (*)"]

        onAccepted: {
            var fileUrlString = saveDialog.file.toString()
            var path = fileUrlString.replace("file://", "")
            configSettings.fileName = path
            configSettings.profile_type = profileSwitch.checked ? 1 : 0
            configSettings.x_disable = xpidDisableCheckBox.checked
            configSettings.x_lock = xpidLockCheckBox.checked
            configSettings.y_disable = ypidDisableCheckBox.checked
            configSettings.y_lock = ypidLockCheckBox.checked
            configSettings.const_throttle = safeParseFloat(constThrottleTextField.text, 0.0)
            configSettings.x_pids = (!xpidDisableCheckBox.checked)
                           ? [configWindow.pidControllers[0][0].value,
                              configWindow.pidControllers[0][1].value,
                              configWindow.pidControllers[0][2].value]
                           : [0, 0, 0]
            configSettings.xpid_saturations = [safeParseFloat(xpidSaturationLowTextField.text, 0.0),
                                               safeParseFloat(xpidSaturationHighTextField.text, 0.0)]
            configSettings.y_pids = (!ypidDisableCheckBox.checked)
                                       ? [configWindow.pidControllers[1][0].value,
                                          configWindow.pidControllers[1][1].value,
                                          configWindow.pidControllers[1][2].value]
                                       : [0, 0, 0]
            configSettings.ypid_saturations = [safeParseFloat(ypidSaturationLowTextField.text, 0.0),
                                               safeParseFloat(ypidSaturationHighTextField.text, 0.0)]
            configSettings.a = safeParseFloat(configWindow.sigmoidCoefficients[0].coefficientTextValue, 0.0)
            configSettings.b = safeParseFloat(configWindow.sigmoidCoefficients[1].coefficientTextValue, 0.0)
            configSettings.size_threshold = safeParseFloat(configWindow.sigmoidCoefficients[2].coefficientTextValue, 0.0)
            configSettings.wing_too_below_threshold = safeParseFloat(configWindow.sigmoidCoefficients[3].coefficientTextValue, 0.0)
            configSettings.wing_too_below_throttle = safeParseFloat(configWindow.sigmoidCoefficients[4].coefficientTextValue, 0.0)
            configSettings.wing_too_above_threshold = safeParseFloat(configWindow.sigmoidCoefficients[5].coefficientTextValue, 0.0)
            configSettings.wing_too_above_throttle = safeParseFloat(configWindow.sigmoidCoefficients[6].coefficientTextValue, 0.0)
            configSettings.wing_tg_at_same_level_throttle = safeParseFloat(configWindow.sigmoidCoefficients[7].coefficientTextValue, 0.0)

            configSettings.sync()
            console.log("Saved config to:", path)
        }
    }

    FileDialog {
        id: loadDialog
        title: "Load Config"
        fileMode: FileDialog.openFile
        nameFilters: ["Config files (*.ini)", "All files (*)"]
        folder: "file:///"

        onAccepted: {
            var fileUrlString = loadDialog.file.toString()
            var path = fileUrlString.replace("file://", "")
            configSettings.fileName = path

            configSettings.sync()

            profileSwitch.checked = configSettings.profile_type === 1
            constThrottleTextField.text = configSettings.const_throttle.toString()
            xpidDisableCheckBox.checked = configSettings.x_disable
            xpidLockCheckBox.checked = configSettings.x_lock
            ypidDisableCheckBox.checked = configSettings.y_disable
            ypidLockCheckBox.checked = configSettings.y_lock
            xpidSaturationLowTextField.text = configSettings.xpid_saturations[0].toString()
            xpidSaturationHighTextField.text = configSettings.xpid_saturations[1].toString()
            ypidSaturationLowTextField.text = configSettings.ypid_saturations[0].toString()
            ypidSaturationHighTextField.text = configSettings.ypid_saturations[1].toString()

            configWindow.pidControllers[0][0].value = configSettings.x_pids[0]
            configWindow.pidControllers[0][1].value = configSettings.x_pids[1]
            configWindow.pidControllers[0][2].value = configSettings.x_pids[2]

            configWindow.pidControllers[1][0].value = configSettings.y_pids[0]
            configWindow.pidControllers[1][1].value = configSettings.y_pids[1]
            configWindow.pidControllers[1][2].value = configSettings.y_pids[2]

            configWindow.sigmoidCoefficients[0].coefficientTextValue = configSettings.a
            configWindow.sigmoidCoefficients[1].coefficientTextValue = configSettings.b
            configWindow.sigmoidCoefficients[2].coefficientTextValue = configSettings.size_threshold
            configWindow.sigmoidCoefficients[3].coefficientTextValue = configSettings.wing_too_below_threshold
            configWindow.sigmoidCoefficients[4].coefficientTextValue = configSettings.wing_too_below_throttle
            configWindow.sigmoidCoefficients[5].coefficientTextValue = configSettings.wing_too_above_threshold
            configWindow.sigmoidCoefficients[6].coefficientTextValue = configSettings.wing_too_above_throttle
            configWindow.sigmoidCoefficients[7].coefficientTextValue = configSettings.wing_tg_at_same_level_throttle

            console.log("Loaded Config from:", path)
        }
    }
}
