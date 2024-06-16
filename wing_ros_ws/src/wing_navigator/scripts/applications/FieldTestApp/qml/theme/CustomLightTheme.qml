import QtQuick 2.15

QtObject {
    readonly property ThemeBase theme: ThemeBase {
        Component.onCompleted: {
            themeMode = "Light";
            primaryColor = "#FFC107";
            accentColor = "#03A9F4";
            foregroundColor = "#5D4037";
            backgroundColor = "#FFF3E0";
        }
    }
}
