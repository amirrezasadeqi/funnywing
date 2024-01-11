import QtLocation 5.15
import QtPositioning 5.15
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtGraphicalEffects 1.15
import "../controls"

Item {
    id: mapWindow
    implicitHeight: 500
    implicitWidth: 500
    property var mapStyles: ["mapbox://styles/mapbox/satellite-streets-v12",
                            "mapbox://styles/mapbox/navigation-night-v1",
                            "mapbox://styles/mapbox/satellite-v9",
                            "mapbox://styles/mapbox/navigation-day-v1"
                            ]
    property int styleIndex: 0
    property string mapBoxAccessToken: "pk.eyJ1IjoiYW1pci1yZXphLXNhZGVnaGkiLCJhIjoiY2xoN2RnMXhhMGY5MDNlbnNlanI0eW45cCJ9.cEA2Vl46PcY5rfWFgURHaA"
    property var mapCenter: QtPositioning.coordinate(35.7471, 51.603)
    property var wingLocation: QtPositioning.coordinate(35.7481, 51.613)
    property var tgLocation: QtPositioning.coordinate(35.745, 51.615)
    property var virtTgLocation: QtPositioning.coordinate(35.7451, 51.615)
    property var wingGoToLocation: QtPositioning.coordinate(35.745, 51.615)
    property real wingGoToAlt: 50.0
    property real wingHdg: 0
    property bool gotoMarkerVisibility: false

    signal sendGoToCommandToBackEnd(real lat, real lon, real alt)

    Rectangle{
        id: mapBg
        anchors.fill: parent

        Map {
            id: map
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.rightMargin: 0
            anchors.leftMargin: 0
            anchors.bottomMargin: 0
            anchors.topMargin: 0

            plugin: Plugin {
                name: "mapboxgl"

                PluginParameter {
                    name: "mapboxgl.mapping.items.insert_before"
                    value: "road-label-small"
                }

                PluginParameter {
                    name: "mapboxgl.access_token"
                    value: mapWindow.mapBoxAccessToken
                }

                PluginParameter {
                    name: "mapboxgl.mapping.additional_style_urls"
                    value: mapWindow.mapStyles[mapWindow.styleIndex]
                }
            }

            center: mapWindow.mapCenter
            zoomLevel: 15
            minimumZoomLevel: 0
            maximumZoomLevel: 20
            tilt: 90

            property int markerSize: 38

            onZoomLevelChanged: {
                map.markerSize = 38 + 8 * (funnywingMarker.zoomLevel - map.zoomLevel);
                if (map.markerSize <= 4){
                    map.markerSize = 4
                }
            }

            copyrightsVisible: false

            MapParameter {
                type: "layer"

                property string name: "3d-buildings"
                property string source: "composite"
                property string sourceLayer: "building"
                property string layerType: "fill-extrusion"
                property real minzoom: 15.0
            }

            MapParameter {
                type: "filter"

                property string layer: "3d-buildings"
                property var filter: [ "==", "extrude", "true" ]
            }

            MapParameter {
                type: "paint"

                property string layer: "3d-buildings"
                property color fillExtrusionColor: "#00617f"
                property real fillExtrusionOpacity: 0.6
                property var fillExtrusionHeight: { return { type: "identity", property: "height" } }
                property var fillExtrusionBase: { return { type: "identity", property: "min_height" } }
            }

            MapQuickItem {
                id: funnywingMarker
                zoomLevel: 15
                coordinate: mapWindow.wingLocation
                anchorPoint{
                    x: funnywingImage.width / 2
                    y: funnywingImage.height / 2
                }
                sourceItem: Image {
                    id: funnywingImage
                    height: map.markerSize
                    width: height
                    source: "../../images/svg_images/switchblade_inair_icon_colorized.svg"
                    sourceSize.height: 875
                    sourceSize.width: 875
                    mipmap: true
                    fillMode: Image.PreserveAspectFit

                    transform: Rotation{
                        id: headingRotation
                        angle: mapWindow.wingHdg
                        origin.x: funnywingMarker.anchorPoint.x
                        origin.y: funnywingMarker.anchorPoint.y
                    }

                    ColorOverlay {
                        id: funnywingColor
                        visible: false
                        source: funnywingImage
                        anchors.fill: funnywingImage
                        color: "green"
                    }
                }
            }

            MapQuickItem {
                id: targetMarker
                zoomLevel: 15
                coordinate: mapWindow.tgLocation
                anchorPoint{
                    x: targetImage.width / 2
                    y: targetImage.height / 2
                }
                sourceItem: Image {
                    id: targetImage
                    height: map.markerSize
                    width: height
                    source: "../../images/svg_images/target_icon.svg"
                    sourceSize.height: 288
                    sourceSize.width: 288

                    ColorOverlay {
                        id: targetColor
                        visible: false
                        source: targetImage
                        anchors.fill: targetImage
                        color: "red"
                    }
                }
            }

            MapQuickItem {
                id: virtualTargetMarker
                zoomLevel: 15
                coordinate: mapWindow.virtTgLocation
                anchorPoint{
                    x: virtualTargetImage.width / 2
                    y: virtualTargetImage.height / 2
                }
                sourceItem: Image {
                    id: virtualTargetImage
                    height: map.markerSize
                    width: height
                    source: "../../images/svg_images/virtual_target_icon.svg"
                    sourceSize.height: 288
                    sourceSize.width: 288

                    ColorOverlay {
                        id: virtualTargetColor
                        visible: false
                        source: virtualTargetImage
                        anchors.fill: virtualTargetImage
                        color: "yellow"
                    }
                }
            }

            MapQuickItem {
                id: gotoMarker
                zoomLevel: 15
                coordinate: mapWindow.wingGoToLocation
                visible: mapWindow.gotoMarkerVisibility
                anchorPoint{
                    x: gotoImage.width / 2
                    y: gotoImage.height
                }
                sourceItem: Image {
                    id: gotoImage
                    height: 24
                    width: 24
                    source: "../../images/svg_images/gotoMarker.svg"

                    ColorOverlay {
                        id: gotoMarkerColor
                        source: gotoImage
                        anchors.fill: gotoImage
                        color: "black"
                    }
                }
            }

            MouseArea{
                id: mapMouseArea
                anchors.fill: parent
                acceptedButtons: Qt.RightButton
                onClicked: {
                    if (mouse.button === Qt.RightButton){
                        mapContextMenu.popup()
                    }
                }

                Menu{
                    id: mapContextMenu
                    MenuItem{
                        text: "Fly to Point"
                        onTriggered: {
                            mapWindow.wingGoToLocation = map.toCoordinate(Qt.point(mapMouseArea.mouseX, mapMouseArea.mouseY))
                            gotoAltInputPopup.open()
                        }
                    }
                }
            }

            Popup{
                id: gotoAltInputPopup
                anchors.centerIn: parent
                width: parent.width * 0.4
                height: 100
                focus: true
                background: Rectangle{
                    id: gotoAltInputPopupBg
                    color: "#2e2f30"
                    anchors.fill: parent
                    radius: 10
                    CustomTextField{
                        id: gotoAltInputPopupTextField
                        width: parent.width - 20
                        anchors.top: parent.top
                        color_on_focus: "#1c2411"
                        color_mouse_hover: "#21261a"
                        default_color: "#2c361d"
                        anchors.topMargin: 15
                        placeholderText: "Enter Command Altitude"
                        anchors.horizontalCenter: parent.horizontalCenter
                        onAccepted: {
                            gotoAltInputPopupBtn.clicked()
                        }
                    }

                    CustomTextBtn{
                        id: gotoAltInputPopupBtn
                        width: 150
                        height: 30
                        btnLabel: "Set Altitude"
                        anchors.top: gotoAltInputPopupTextField.bottom
                        anchors.topMargin: 10
                        anchors.horizontalCenter: parent.horizontalCenter
                        onClicked: {
                            mapWindow.wingGoToAlt = parseFloat(gotoAltInputPopupTextField.text)
                            mapWindow.sendGoToCommandToBackEnd(mapWindow.wingGoToLocation.latitude, mapWindow.wingGoToLocation.longitude, mapWindow.wingGoToAlt)
                            gotoAltInputPopup.close()
                            mapWindow.gotoMarkerVisibility = true
                        }
                    }
                }
            }
        }
    }
}
