import QtQuick 2.15
import QtQuick.Controls.Material 2.15

pragma Singleton

Item {
    id: themeManager

    property string themeName: "customLight"
    // This property is for the user components. The Material here is for internal ThemeManager internal usage
    // and when we get properties from the proxy this returns the value. So, we must also attach the Material
    // in the user Components and for example if we don't attach it to the main.qml file root component, the
    // Material.theme changes only apply in this file, not in the whole application.
    readonly property int materialTheme: Material.theme

    // m3 material design color roles: m3.material.io
    // use ThemeManager.m3["primary"] for example to get your theme's primary.
    property var m3: themeLoader.item.theme.m3

    property var registeredComponents: []

    ThemeFileMap {
        id: themeFileMap
    }

    function getProperty(name) {
        if (themeManager.hasOwnProperty(name)) {
            return themeManager[name];
        } else if (Material.hasOwnProperty(name)) {
            return Material[name];
        } else {
            console.log("Property " + name + " does not exist in ThemeManager or Material");
            return undefined;
        }
    }

    property var proxy: new Proxy(themeManager, {
        get: function(target, name) {
            return target.getProperty(name);
        }
    })

    function color(clr){
        if (themeManager.hasOwnProperty(clr)){
            return themeManager[clr];
        } else if (Material.hasOwnProperty(clr)){
            return Material.color(Material[clr]);
        } else {
            console.log("Color " + clr + " does not exist in ThemeManager or Material.");
            return themeManager.proxy.primary;
        }
    }

    function applyTheming(themeObj) {
        // Sets up the Material theme attached to the ThemeManager. themeObj is the theme template
        // defined in <something>Theme.qml files.
        Material.theme = themeObj.theme.themeMode === "Light" ? Material.Light : Material.Dark;
        Material.primary = themeObj.theme.primaryColor;
        Material.accent = themeObj.theme.accentColor;
        Material.foreground = themeObj.theme.foregroundColor;
        Material.background = themeObj.theme.backgroundColor;
    }

    function setupComponentTheme(component) {
        // Gets a component(any component, even outside of the ThemeManager) and applies the Material configs attached
        // to the ThemeManager. This attached Material is configured in the applyTheming function.
        component.Material.theme = Material.theme;
        component.Material.primary = Material.primary;
        component.Material.accent = Material.accent;
        component.Material.foreground = Material.foreground;
        component.Material.background = Material.background;
    }

    function register(component) {
        registeredComponents.push(component);
        setupComponentTheme(component);
    }

    function updateComponentsTheme() {
        for (var i = 0; i < registeredComponents.length; i++) {
            setupComponentTheme(registeredComponents[i]);
        }
    }

    Loader {
        id: themeLoader
        source: themeFileMap.themeFileMapDict[themeName]
        onLoaded: {
            applyTheming(themeLoader.item);
        }
    }

    onThemeNameChanged: {
        themeLoader.source = themeFileMap.themeFileMapDict[themeName];
        updateComponentsTheme();
    }
}
