import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Window 2.15
import QtGraphicalEffects 1.15
import QtPositioning 5.15
import "controls"
import "pages"

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
    }

    Rectangle {
        id: bg
        x: 480
        y: 173
        width: 200
        height: 200
        color: "#2a2a2a"
        anchors.fill: parent

        Rectangle {
            id: appContainer
            x: 541
            y: 182
            width: 200
            height: 200
            color: "#002a2a2a"
            anchors.fill: parent

            Rectangle {
                id: topBar
                height: 60
                color: "#232323"
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                Rectangle{
                    id: appIconBg
                    width: 50
                    height: width
                    color: iconBgColor
                    radius: width / 2
                    anchors.left: parent.left
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    property url switchBlade: "../images/svg_images/switchblade_launch_icon.svg"
                    property color iconBgColor: "#98A8BB"

                    MouseArea{
                        id: appIconMouseArea
                        anchors.fill: parent
                        hoverEnabled: true
                        onEntered: {
                            appIconBg.switchBlade = "../images/svg_images/switchblade_inair_icon.svg"
                            appIconBg.iconBgColor = "#bbb6a3"
                        }
                        onExited: {
                            appIconBg.switchBlade = "../images/svg_images/switchblade_launch_icon.svg"
                            appIconBg.iconBgColor = "#98A8BB"
                        }
                    }

                    Image {
                        id: appIcon
                        anchors.fill: parent
                        source: appIconBg.switchBlade
                        sourceSize.height: 512
                        sourceSize.width: 512
                        fillMode: Image.PreserveAspectFit

                        ColorOverlay{
                            anchors.fill: parent
                            source: appIcon
                            color: "#675F51"
                        }
                    }
                }

                Rectangle {
                    id: topBarContainer
                    color: "#00ffffff"
                    anchors.left: appIconBg.right
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.leftMargin: 10
                }
            }

            Rectangle {
                id: sideBarContainer
                color: "#00ffffff"
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: topBar.bottom
                anchors.bottom: parent.bottom
                anchors.topMargin: 0

                Rectangle {
                    id: leftBar
                    width: 40
                    color: "#232323"
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom

                    Rectangle {
                        id: dataDisplaySideContainer
                        height: baseContentTopContainer.height
                        color: "#00ffffff"
                        anchors{
                            top: parent.top
                            left: parent.left
                            right: parent.right
                        }

                        Label {
                            id: dataMonitorTitle
                            color: "#dddddd"
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
                        color: "#2a2a2a"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: dataDisplaySideContainer.bottom
                        anchors.topMargin: 0
                    }

                    Rectangle {
                        id: actionsSideContainer
                        color: "#00ffffff"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: sideBarTitleSeparator.bottom
                        anchors.bottom: parent.bottom
                        anchors.topMargin: 0

                        Label {
                            id: actionPanelTitle
                            color: "#dddddd"
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
                    color: "#00ffffff"
                    anchors.left: leftBar.right
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.leftMargin: 0

                    Rectangle {
                        id: bottomBar
                        y: 302
                        height: 30
                        color: "#232323"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                    }

                    Rectangle {
                        id: baseContent
                        color: "#00ffffff"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: bottomBar.top
                        anchors.bottomMargin: 0

                        Rectangle {
                            id: baseContentTopContainer
                            color: "#00ffffff"
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.bottom: baseContentBottomContainer.top
                            anchors.bottomMargin: 0

                            Rectangle {
                                id: dataDisplayContainer
                                width: baseContentTopContainer.width * 0.45
                                color: "#e8232323"
                                anchors.left: parent.left
                                anchors.top: parent.top
                                anchors.bottom: parent.bottom
                                Rectangle {
                                    id: dataMonitorTabContainer
                                    height: 41
                                    color: "#ffffff"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: parent.top
                                    TabBar {
                                        id: monitorBtnBar
                                        width: parent.width
                                        currentIndex: monitorSwipeView.currentIndex
                                        TabButton{
                                            id: primaryDataMonitorBtn
                                            text: qsTr("Primary Data")
                                        }
                                        TabButton{
                                            id: otherMonitors
                                            text: qsTr('Other Monitors')
                                        }
                                    }
                                }

                                Rectangle {
                                    id: monitorSwipeViewContainer
                                    color: "#00ffffff"
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
                                            }
                                        }

                                        Item {
                                            id: otherMonitorTab
                                            Rectangle{
                                                id: otherMonitorContainer
                                                anchors.fill: parent
                                                color: "transparent"
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
                                color: "#00ffffff"
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
                                    height: 30
                                    color: "#00ffffff"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.bottom: parent.bottom
                                    anchors.rightMargin: 0
                                    anchors.leftMargin: 0
                                    anchors.bottomMargin: 0

                                    CustomTextBtn {
                                        id: moveToWingBtn
                                        width: 80
                                        height: 28
                                        anchors.verticalCenter: parent.verticalCenter
                                        anchors.left: parent.left
                                        btnLabel: "Wing"
                                        anchors.leftMargin: 5
                                        onClicked: {
                                            map.mapCenter = map.wingLocation
                                        }
                                    }

                                    CustomTextBtn {
                                        id: moveToTgBtn
                                        width: 80
                                        height: 28
                                        anchors.verticalCenter: parent.verticalCenter
                                        anchors.left: moveToWingBtn.right
                                        btnLabel: "Target"
                                        anchors.leftMargin: 5
                                        onClicked: {
                                            map.mapCenter = map.tgLocation
                                        }
                                    }

                                    CustomTextBtn {
                                        id: clearBtn
                                        width: 100
                                        height: 28
                                        anchors.verticalCenter: parent.verticalCenter
                                        anchors.right: parent.right
                                        btnLabel: "Clear Map"
                                        anchors.rightMargin: 5
                                        onClicked: {
                                            map.clearMap();
                                        }
                                    }
                                }

                                Rectangle {
                                    id: mapDataDisplayerContainer
                                    width: 160
                                    height: 50
                                    color: "#9938383c"
                                    border.width: 3
                                    border.color: "#38383c"
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
                                        color: "red"
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
                                        color: "red"
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
                                        color: "red"
                                        anchors {
                                            top: wingFlightMode.bottom
                                            topMargin: 5
                                            left: wingRelAltLabel.right
                                            leftMargin: 5
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
                                color: "#00ffffff"
                                anchors.left: parent.left
                                anchors.right: parent.right
                                anchors.top: baseContentSeparator.bottom
                                anchors.bottom: parent.bottom
                                anchors.topMargin: 0

                                Rectangle {
                                    id: actionTabcontainer
                                    height: 41
                                    color: "#ffffff"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: parent.top
                                    TabBar {
                                        id: actionBtnBar
                                        width: parent.width
                                        currentIndex: actionSwipeView.currentIndex
                                        TabButton{
                                            id: stateActionBtn
                                            text: qsTr("State Actions")
                                        }
                                        TabButton{
                                            id: gotoServiceBtn
                                            text: qsTr('Goto Service')
                                        }
                                    }
                                }

                                Rectangle {
                                    id: actionSwipeViewContainer
                                    color: "#00ffffff"
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: actionTabcontainer.bottom
                                    anchors.bottom: parent.bottom
                                    anchors.topMargin: 0

                                    SwipeView {
                                        id: actionSwipeView
                                        anchors.fill: parent
                                        clip: true
                                        currentIndex: actionBtnBar.currentIndex

                                        Item {
                                            id: stateActionTab
                                            StateActionView{
                                                id: stateActionView
                                                anchors.fill: parent
                                                onArmDisarmBtnSignal: {
                                                    backFrontConnections.setArmState(arming)
                                                }
                                                onModeChangerBtnsSignal: {
                                                    backFrontConnections.setFlightMode(mode)
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



