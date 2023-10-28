import QtLocation 5.15
import QtPositioning 5.15
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtGraphicalEffects 1.15


Item {
    id: mapWindow
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
    property real wingHdg: 0

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
                    height: 38
                    width: 38
                    source: "../../images/svg_images/switchblade_inair_icon.svg"

                    transform: Rotation{
                        id: headingRotation
                        angle: mapWindow.wingHdg
                        origin.x: funnywingMarker.anchorPoint.x
                        origin.y: funnywingMarker.anchorPoint.y
                    }

                    ColorOverlay {
                        id: funnywingColor
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
                    height: 33
                    width: 33
                    source: "../../images/svg_images/close_icon.svg"

                    ColorOverlay {
                        id: targetColor
                        source: targetImage
                        anchors.fill: targetImage
                        color: "red"
                    }
                }
            }

        }
    }
}
