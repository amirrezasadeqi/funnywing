import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Window 2.15
import QtGraphicalEffects 1.15
import QtPositioning 5.15
import "controls"
import "pages"
import "theme" 1.0

Window {
    id: mainWindow
    width: 1250
    height: 680
    visible: true
    color: "#00000000"
    minimumHeight: 400
    minimumWidth: 800
    title: qsTr("Field Test App")

    property var tgGPS: {'lat': 35.7480, 'lon': 51.603, 'alt': 1382.454545}
    property var virtTgGPS: {'lat': 35.7481, 'lon': 51.603, 'alt': 1382.454545}
    property var wingGPS: {'lat': 35.7471, 'lon': 51.603, 'alt': 1382.454545}
    property var wingVel: {'vx': -11.454646, 'vy': 2.8777544, 'vz': 0.4565454}
    property real wingHdg: 90
    property string wingFlightState: "GUIDED"
    property real wingRelAlt: 100.432
    property real distToTg: 54.12
    property string rescueStatus: "OFF"
    property real tgRecvDataRate: 5.0
    property real wingRecvDataRate: 10.0
    property real tgRelAlt: 50.0

    Connections{
        target: backFrontConnections

        function onSetTargetGPS(lat, lon, alt){
            mainWindow.tgGPS = {'lat': lat, 'lon': lon, 'alt': alt}
        }
        function onSetVirtualTargetGPS(lat, lon, alt){
            mainWindow.virtTgGPS = {'lat': lat, 'lon': lon, 'alt': alt}
        }
        function onSetWingGPS(lat, lon, alt){
            mainWindow.wingGPS = {'lat': lat, 'lon': lon, 'alt': alt}
        }
        function onSetWingVelocity(vx, vy, vz){
            mainWindow.wingVel = {'vx': vx, 'vy': vy, 'vz': vz}
        }
        function onSetWingHeading(hdg){
            mainWindow.wingHdg = hdg
        }
        function onSetWingFlightState(flightState){
            mainWindow.wingFlightState = flightState
        }
        function onSetWingRelAlt(alt){
            mainWindow.wingRelAlt = alt
        }
        function onSetDistanceToTarget(dist){
            mainWindow.distToTg = dist
        }
        function onShowRescueStatus(rescueStatus){
            mainWindow.rescueStatus = rescueStatus ? "ON" : "OFF";
        }
        function onSetWingRecvDataRate(rate){
            mainWindow.wingRecvDataRate = rate
        }
        function onSetTgRecvDataRate(rate){
            mainWindow.tgRecvDataRate = rate
        }
        function onUpdateCameraMonitorFrame(){
            cameraMonitorOutPut.reload()
        }
    }

    // Ignore the Invalid property name "onClosing". (M16) error. This is a bug of the IDE.
    onClosing: function(close) {
        // If you need to clean up the backend before the window can be closed, so preventing from segfault erros
        // caused by sending signals from the backend to destroyed frontend slots(I think vice versa.).
        close.accepted = false
        // QQmlApplicationEngine does not send quit signal automaticaly, so send it to close the back-end.
        backFrontConnections.closeBackend()
    }

    Rectangle {
        id: bg
        x: 480
        y: 173
        width: 200
        height: 200
        color: ThemeManager.m3["surface"]
        anchors.fill: parent

        Rectangle {
            id: appContainer
            x: 541
            y: 182
            width: 200
            height: 200
            color: "transparent"
            anchors.fill: parent

            Rectangle {
                id: topBar
                height: 60
                color: ThemeManager.m3["surfaceContainerHighest"]
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top

                Rectangle {
                    id: appIconContainer
                    width: 50
                    anchors {
                        left: parent.left
                        leftMargin: 10
                        verticalCenter: parent.verticalCenter
                    }
                    color: "transparent"
                    RoundButton {
                        id: appIconBtn
                        anchors.centerIn: parent
                        radius: 50
                        Material.background: ThemeManager.m3["secondaryContainer"]
                        Material.elevation: 2
                        icon.source: hovered ? "../images/svg_images/switchblade_inair_icon.svg" : "../images/svg_images/switchblade_launch_icon.svg"
                        icon.color: ThemeManager.m3["onSecondaryContainer"]
                        icon.width: 40
                        icon.height: 40
                    }
                }

                Rectangle {
                    id: topBarContainer
                    color: "transparent"
                    anchors.left: appIconBg.right
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.leftMargin: 10

                    RoundButton {
                        id: themeSwitchBtn
                        height: 0.85 * parent.height
                        width: height
                        Material.background: ThemeManager.m3["secondaryContainer"]

                        anchors {
                            right: parent.right
                            rightMargin: 10
                            verticalCenter: parent.verticalCenter
                        }

                        Image {
                            id: themeSwitchBtnIcon
                            source: "../images/svg_images/darkModeIcon.svg"
                            anchors.centerIn: parent
                            width: 0.5 * parent.width
                            height: 0.5 * parent.height
                        }

                        onClicked: {
                            ThemeManager.themeName = ThemeManager.themeName === "customLight" ? "customDark" : "customLight";
                            themeSwitchBtnIcon.source = ThemeManager.themeName === "customLight" ? "../images/svg_images/darkModeIcon.svg" : "../images/svg_images/lightModeIcon.svg";
                        }
                    }
                }
            }

            Rectangle {
                id: sideBarContainer
                color: "transparent"
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: topBar.bottom
                anchors.bottom: parent.bottom
                anchors.topMargin: 0

                Rectangle {
                    id: leftBar
                    width: 40
                    color: ThemeManager.m3["surfaceContainerHighest"]
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom

                    Rectangle {
                        id: dataDisplaySideContainer
                        height: baseContentTopContainer.height
                        color: "transparent"
                        anchors{
                            top: parent.top
                            left: parent.left
                            right: parent.right
                        }

                        Label {
                            id: dataMonitorTitle
                            color: ThemeManager.m3["onSurface"]
                            text: qsTr("Monitor Panel")
                            anchors.verticalCenter: parent.verticalCenter
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.family: "Times New Roman"
                            font.styleName: "Bold"
                            antialiasing: true
                            renderType: Text.QtRendering
                            font.pointSize: 19
                            scale: 1
                            anchors.horizontalCenter: parent.horizontalCenter
                            rotation: -90
                            clip: false
                            font.weight: Font.Bold
                        }
                    }

                    Rectangle {
                        id: sideBarTitleSeparator
                        height: baseContentSeparator.height * 0.2
                        color: ThemeManager.m3["outlineVariant"]
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: dataDisplaySideContainer.bottom
                        anchors.topMargin: 0
                    }

                    Rectangle {
                        id: actionsSideContainer
                        color: "transparent"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: sideBarTitleSeparator.bottom
                        anchors.bottom: parent.bottom
                        anchors.topMargin: 0

                        Label {
                            id: actionPanelTitle
                            color: ThemeManager.m3["onSurface"]
                            text: qsTr("Action Panel")
                            anchors.verticalCenter: parent.verticalCenter
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.family: "Times New Roman"
                            font.styleName: "Bold"
                            antialiasing: true
                            renderType: Text.QtRendering
                            font.pointSize: 19
                            scale: 1
                            anchors.horizontalCenter: parent.horizontalCenter
                            rotation: -90
                            clip: false
                            font.weight: Font.Bold
                        }
                    }
                }

                Rectangle {
                    id: sideBarRightContainer
                    color: "transparent"
                    anchors.left: leftBar.right
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.leftMargin: 0

                    Rectangle {
                        id: bottomBar
                        y: 302
                        height: 30
                        color: ThemeManager.m3["surfaceContainerHighest"]
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                    }

                    Rectangle {
                        id: baseContent
                        color: "transparent"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: bottomBar.top
                        anchors.bottomMargin: 0

                        Rectangle {
                            id: baseContentTopContainer
                            color: "transparent"
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.bottom: baseContentBottomContainer.top
                            anchors.bottomMargin: 0

                            Rectangle {
                                id: dataDisplayContainer
                                width: baseContentTopContainer.width * 0.45
                                color: "transparent"
                                anchors.left: parent.left
                                anchors.top: parent.top
                                anchors.bottom: parent.bottom
                                Rectangle {
                                    id: dataMonitorTabContainer
                                    height: 40
                                    color: "transparent"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: parent.top
                                    TabBar {
                                        id: monitorBtnBar
                                        anchors.fill: parent
                                        currentIndex: monitorSwipeView.currentIndex
                                        Material.background: ThemeManager.materialTheme === Material.Light ? Qt.lighter(ThemeManager.m3["secondaryContainer"], 2.5) : ThemeManager.m3["secondaryContainer"]
                                        Material.accent: ThemeManager.m3["tertiary"]
                                        TabButton{
                                            id: primaryDataMonitorBtn
                                            anchors {
                                                top: parent.top
                                                bottom: parent.bottom
                                            }
                                            contentItem: Text {
                                                id: primaryDataMonitorBtnText
                                                horizontalAlignment: Text.AlignHCenter
                                                text: qsTr("Primary Data")
                                                font.bold: true
                                                color: ThemeManager.m3["onSecondaryContainer"]
                                            }
                                        }
                                        TabButton{
                                            id: cameraMonitorBtn
                                            anchors {
                                                top: parent.top
                                                bottom: parent.bottom
                                            }
                                            contentItem: Text {
                                                id: cameraMonitorBtnText
                                                horizontalAlignment: Text.AlignHCenter
                                                text: qsTr("Camera")
                                                font.bold: true
                                                color: ThemeManager.m3["onSecondaryContainer"]
                                            }
                                        }
                                    }
                                }

                                Rectangle {
                                    id: monitorSwipeViewContainer
                                    color: "transparent"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: dataMonitorTabContainer.bottom
                                    anchors.bottom: parent.bottom
                                    anchors.topMargin: 0
                                    SwipeView{
                                        id: monitorSwipeView
                                        anchors.fill: parent
                                        currentIndex: monitorBtnBar.currentIndex
                                        clip: true
                                        Item {
                                            id: primaryDataMonitorTab
                                            clip: true
                                            PrimaryDataView{
                                                id: primaryDataView
                                                anchors.fill: parent
                                                heading: mainWindow.wingHdg
                                                wingFlightState: mainWindow.wingFlightState
                                                wingRelativeAlt: mainWindow.wingRelAlt
                                                wingGpsLatVal: mainWindow.wingGPS.lat
                                                wingGpsLonVal: mainWindow.wingGPS.lon
                                                wingGpsAltVal: mainWindow.wingGPS.alt
                                                wingVelValX: mainWindow.wingVel.vx
                                                wingVelValY: mainWindow.wingVel.vy
                                                wingVelValZ: mainWindow.wingVel.vz
                                                tgGpsLatVal: mainWindow.tgGPS.lat
                                                tgGpsLonVal: mainWindow.tgGPS.lon
                                                tgGpsAltVal: mainWindow.tgGPS.alt
                                                distToTg: mainWindow.distToTg
                                                tgRecvDataRate: mainWindow.tgRecvDataRate
                                                tgRelAlt: mainWindow.tgRelAlt
                                            }
                                        }

                                        Item {
                                            id: cameraMonitorTab
                                            Rectangle{
                                                id: cameraMonitorContainer
                                                anchors.fill: parent
                                                color: "transparent"

                                                Image {
                                                    id: cameraMonitorOutPut
                                                    property bool frameFlipper: false
                                                    source: "image://cameraMonitorFrameProvider/frame"
                                                    anchors.fill: parent
                                                    cache: false
                                                    fillMode: Image.PreserveAspectFit

                                                    function reload() {
                                                        // This is just for change in the source name, to force the image to reload.
                                                        frameFlipper = !frameFlipper
                                                        source = "image://cameraMonitorFrameProvider/frame?id=" + frameFlipper
                                                    }
                                                }
                                            }
                                        }
                                    }

                                    PageIndicator {
                                        id: monitorPageIndicator
                                        count: monitorSwipeView.count
                                        currentIndex: monitorSwipeView.currentIndex
                                        anchors.bottom: monitorSwipeView.bottom
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                }
                            }

                            Rectangle {
                                id: mapContainer
                                color: "transparent"
                                anchors.left: dataDisplayContainer.right
                                anchors.right: parent.right
                                anchors.top: parent.top
                                anchors.bottom: parent.bottom
                                anchors.leftMargin: 0
                                MapView{
                                    id: map
                                    anchors.fill: parent
                                    anchors.leftMargin: 3
                                    wingLocation: QtPositioning.coordinate(mainWindow.wingGPS.lat, mainWindow.wingGPS.lon)
                                    tgLocation: QtPositioning.coordinate(mainWindow.tgGPS.lat, mainWindow.tgGPS.lon)
                                    virtTgLocation: QtPositioning.coordinate(mainWindow.virtTgGPS.lat, mainWindow.virtTgGPS.lon)
                                    wingHdg: mainWindow.wingHdg
                                    onSendGoToCommandToBackEnd: {
                                        backFrontConnections.goToLocation(lat, lon, alt)
                                    }
                                }

                                Rectangle {
                                    id: mapHorizontalControlsContainer
                                    height: 60
                                    color: "transparent"
                                    anchors {
                                        left: map.left
                                        right: parent.right
                                        bottom: parent.bottom
                                    }
                                    RoundButton {
                                        id: moveToWingBtn
                                        width: 50
                                        height: 50
                                        opacity: 0.9
                                        Material.background: ThemeManager.m3["tertiaryContainer"]
                                        anchors {
                                            verticalCenter: parent.verticalCenter
                                            left: parent.left
                                            leftMargin: 5
                                        }
                                        text: qsTr("Wing")
                                        icon {
                                            source: "../images/svg_images/switchblade_inair_icon.svg"
                                            color: ThemeManager.m3["onTertiaryContainer"]
                                            width: 50
                                            height: 50
                                        }
                                        onClicked: {
                                            map.mapCenter = map.wingLocation
                                        }
                                    }

                                    RoundButton {
                                        id: moveToTgBtn
                                        width: 50
                                        height: 50
                                        opacity: 0.9
                                        Material.background: ThemeManager.m3["tertiaryContainer"]
                                        anchors {
                                            verticalCenter: parent.verticalCenter
                                            left: moveToWingBtn.right
                                            leftMargin: 5
                                        }
                                        icon {
                                            source: "../images/svg_images/goToTargetIcon.svg"
                                            color: ThemeManager.m3["onTertiaryContainer"]
                                            width: 40
                                            height: 40
                                        }
                                        onClicked: {
                                            map.mapCenter = map.tgLocation
                                        }
                                    }

                                    RoundButton {
                                        id: clearBtn
                                        width: 50
                                        height: 50
                                        opacity: 0.9
                                        Material.background: ThemeManager.m3["tertiaryContainer"]
                                        anchors {
                                            verticalCenter: parent.verticalCenter
                                            right: parent.right
                                            rightMargin: 5
                                        }
                                        icon {
                                            source: "../images/png_images/clearIcon.png"
                                            color: ThemeManager.m3["onTertiaryContainer"]
                                            width: 20
                                            height: 20
                                        }
                                        onClicked: {
                                            map.clearMap();
                                        }
                                    }

                                    RoundButton {
                                        id: rescueOnBtn
                                        width: 50
                                        height: 50
                                        opacity: 0.9
                                        Material.background: ThemeManager.m3["tertiaryContainer"]
                                        anchors {
                                            verticalCenter: parent.verticalCenter
                                            right: clearBtn.left
                                            rightMargin: 5
                                        }
                                        icon {
                                            source: "../images/png_images/protectedIcon.png"
                                            color: ThemeManager.m3["onTertiaryContainer"]
                                            width: 20
                                            height: 20
                                        }
                                        onClicked: {
                                            backFrontConnections.sendSetRescueStatus(true);
                                        }
                                    }

                                    RoundButton {
                                        id: rescueOffBtn
                                        width: 50
                                        height: 50
                                        opacity: 0.9
                                        Material.background: ThemeManager.m3["tertiaryContainer"]
                                        anchors {
                                            verticalCenter: parent.verticalCenter
                                            right: rescueOnBtn.left
                                            rightMargin: 5
                                        }
                                        icon {
                                            source: "../images/png_images/unprotectedIcon.png"
                                            color: ThemeManager.m3["onTertiaryContainer"]
                                            width: 20
                                            height: 20
                                        }
                                        onClicked: {
                                            backFrontConnections.sendSetRescueStatus(false);
                                        }
                                    }
                                }

                                Rectangle {
                                    id: mapDataDisplayerContainer
                                    width: 160
                                    height: 115
                                    color: ThemeManager.m3["surfaceContainer"]
                                    opacity: 0.8
                                    border {
                                        color: ThemeManager.m3["outlineVariant"]
                                        width: 3
                                    }
                                    clip: true
                                    anchors {
                                        left: parent.left
                                        top: parent.top
                                        leftMargin: 5
                                        topMargin: 5
                                    }
                                    Label {
                                        id: wingFlightMode
                                        text: mainWindow.wingFlightState
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: parent.top
                                            topMargin: 5
                                            left: parent.left
                                            leftMargin: 5
                                        }
                                    }
                                    Label {
                                        id: wingRelAltLabel
                                        text: "Wing Rel Alt: "
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: wingFlightMode.bottom
                                            topMargin: 5
                                            left: parent.left
                                            leftMargin: 5
                                        }
                                    }
                                    Label {
                                        id: wingRelAltValueLabel
                                        text: mainWindow.wingRelAlt
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: wingFlightMode.bottom
                                            topMargin: 5
                                            left: wingRelAltLabel.right
                                            leftMargin: 0
                                        }
                                    }
                                    Label {
                                        id: rescueStateLabel
                                        text: "Rescue: "
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: wingRelAltLabel.bottom
                                            topMargin: 5
                                            left: parent.left
                                            leftMargin: 5
                                        }
                                    }
                                    Label {
                                        id: rescueStateValueLabel
                                        text: mainWindow.rescueStatus
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: wingRelAltLabel.bottom
                                            topMargin: 5
                                            left: rescueStateLabel.right
                                            leftMargin: 0
                                        }
                                    }
                                    Label {
                                        id: wingRecvDataRateLabel
                                        text: "Wing Data Rate: "
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: rescueStateLabel.bottom
                                            topMargin: 5
                                            left: parent.left
                                            leftMargin: 5
                                        }
                                    }
                                    Label {
                                        id: wingRecvDataRateValueLabel
                                        text: mainWindow.wingRecvDataRate.toFixed(2)
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: rescueStateLabel.bottom
                                            topMargin: 5
                                            left: wingRecvDataRateLabel.right
                                            leftMargin: 0
                                        }
                                    }
                                    Label {
                                        id: tgRecvDataRateLabel
                                        text: "Target Data Rate: "
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: wingRecvDataRateLabel.bottom
                                            topMargin: 5
                                            left: parent.left
                                            leftMargin: 5
                                        }
                                    }
                                    Label {
                                        id: tgRecvDataRateValueLabel
                                        text: mainWindow.tgRecvDataRate.toFixed(2)
                                        color: ThemeManager.m3["onSurface"]
                                        anchors {
                                            top: wingRecvDataRateLabel.bottom
                                            topMargin: 5
                                            left: tgRecvDataRateLabel.right
                                            leftMargin: 0
                                        }
                                    }
                                }
                            }
                        }

                        Rectangle {
                            id: baseContentBottomContainer
                            y: 334
                            height: parent.height * 0.4
                            color: "#00ffffff"
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.bottom: parent.bottom

                            Rectangle {
                                id: baseContentSeparator
                                height: 20
                                color: "#232323"
                                anchors.left: parent.left
                                anchors.right: parent.right
                                anchors.top: parent.top
                            }

                            Rectangle {
                                id: bottomContainer
                                color: "transparent"
                                anchors.left: parent.left
                                anchors.right: parent.right
                                anchors.top: baseContentSeparator.bottom
                                anchors.bottom: parent.bottom
                                anchors.topMargin: 0

                                Rectangle {
                                    id: actionTabcontainer
                                    height: 40
                                    color: "transparent"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: parent.top
                                    TabBar {
                                        id: actionBtnBar
                                        anchors.fill: parent
                                        currentIndex: actionSwipeView.currentIndex
                                        Material.background: ThemeManager.materialTheme === Material.Light ? Qt.lighter(ThemeManager.m3["secondaryContainer"], 2.5) : ThemeManager.m3["secondaryContainer"]
                                        Material.accent: ThemeManager.m3["tertiary"]
                                        TabButton{
                                            id: stateActionBtn
                                            anchors {
                                                top: parent.top
                                                bottom: parent.bottom
                                            }
                                            contentItem: Text {
                                                id: stateActionBtnText
                                                horizontalAlignment: Text.AlignHCenter
                                                text: qsTr("Actions")
                                                font.bold: true
                                                color: ThemeManager.m3["onSecondaryContainer"]
                                            }
                                        }
                                        TabButton{
                                            id: configPageBtn
                                            anchors {
                                                top: parent.top
                                                bottom: parent.bottom
                                            }
                                            contentItem: Text {
                                                id: configPageBtnText
                                                horizontalAlignment: Text.AlignHCenter
                                                text: qsTr("Configuration")
                                                font.bold: true
                                                color: ThemeManager.m3["onSecondaryContainer"]
                                            }
                                        }
                                        TabButton{
                                            id: gotoServiceBtn
                                            anchors {
                                                top: parent.top
                                                bottom: parent.bottom
                                            }
                                            contentItem: Text {
                                                id: gotoServiceBtnText
                                                horizontalAlignment: Text.AlignHCenter
                                                text: qsTr("Services")
                                                font.bold: true
                                                color: ThemeManager.m3["onSecondaryContainer"]
                                            }
                                        }
                                    }
                                }

                                Rectangle {
                                    id: actionSwipeViewContainer
                                    color: "transparent"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: actionTabcontainer.bottom
                                    anchors.bottom: parent.bottom
                                    SwipeView {
                                        id: actionSwipeView
                                        anchors.fill: parent
                                        clip: true
                                        currentIndex: actionBtnBar.currentIndex

                                        Item {
                                            id: stateActionTab
                                            ActionView{
                                                id: actionView
                                                anchors.fill: parent
                                                onArmDisarmBtnSignal: {
                                                    backFrontConnections.setArmState(arming);
                                                }
                                                onModeChangerBtnsSignal: {
                                                    backFrontConnections.setFlightMode(mode);
                                                }
                                                onTestScenarioBtnSignal: {
                                                    backFrontConnections.handleTestScenario(scenarioIdx, active);
                                                }
                                                onSetSettingsBtnSignal: {
                                                    backFrontConnections.setSimpleTrackerSettings(waypointRadius, local, wingAsVirtualCenter);
                                                }
                                                onSimpleTrackerBtnsSignal: {
                                                    backFrontConnections.setSimpleTrackerActivation(active);
                                                }
                                                onSetApParamBtnSignal: {
                                                    backFrontConnections.setArduplaneParam(paramName, paramValue);
                                                }
                                            }
                                        }

                                        Item {
                                            id: configurationView
                                            Rectangle{
                                                id: configurationContainer
                                                anchors.fill: parent
                                                color: "transparent"

                                                Button {
                                                    id: testButton
                                                    text: qsTr("Material themed Button")
                                                    anchors.centerIn: parent
                                                    width: 300
                                                    height: 100
                                                    Component.onCompleted: {
                                                        ThemeManager.register(testButton);
                                                    }
                                                }

                                                Button {
                                                    id: anotherTestBtn
                                                    width: 200
                                                    height: 30
                                                    anchors {
                                                        left: testButton.right
                                                        verticalCenter: testButton.verticalCenter
                                                        leftMargin: 10
                                                    }

                                                    background: Rectangle {
                                                        id: anotherTestBtnRect
                                                        anchors.fill: parent
                                                        color: ThemeManager.m3["secondary"]
                                                        Text {
                                                            id: textTest
                                                            anchors.centerIn: parent
                                                            text: qsTr("Another Material Button")
                                                            font.styleName: "Bold"
                                                            color: ThemeManager.m3["onSecondary"]
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        Item {
                                            id: gotoServiceTab
                                            GotoServiceView{
                                                id: gotoServiceView
                                                anchors.fill: parent
                                                onGoToSignal: {
                                                    backFrontConnections.goToLocation(lat, lon, alt)
                                                }
                                            }
                                        }
                                    }

                                    PageIndicator {
                                        id: actionPageIndicator
                                        count: actionSwipeView.count
                                        currentIndex: actionSwipeView.currentIndex
                                        anchors.bottom: actionSwipeView.bottom
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}



