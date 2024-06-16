import QtQuick 2.15

QtObject {
    readonly property ThemeBase theme: ThemeBase {
        Component.onCompleted: {
            themeMode = "Dark";
            primaryColor = "#FFA000";
            accentColor = "#FFC107";
            foregroundColor = "#FFFFFF";
            backgroundColor = "#263238";
        }
    }
}
