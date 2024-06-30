import QtQuick 2.15

QtObject {
    id: theme

    // Corresponding properties to QML material module configurable properties
    property string themeMode: "Light"
    property color primaryColor: "#00224A"  // primary in m3 design
    property color accentColor: "#FFFFFF"  // does not have exact correspondence in m3 design
    property color foregroundColor: "#FFFFFF"  // onPrimary and ... in m3 material
    property color backgroundColor: "#00224A"  // background of elements like buttos. equivalent to primary in m3 material

    // Inspired properties from m3 material design: m3.material.io
    property var m3: {
        "primary": "#00224A",
        "surfaceTint": "#405F91",
        "onPrimary": "#FFFFFF",
        "primaryContainer": "#234373",
        "onPrimaryContainer": "#FFFFFF",
        "secondary": "#192232",
        "onSecondary": "#FFFFFF",
        "secondaryContainer": "#3A4354",
        "onSecondaryContainer": "#FFFFFF",
        "tertiary": "#2F1A35",
        "onTertiary": "#FFFFFF",
        "tertiaryContainer": "#523A58",
        "onTertiaryContainer": "#FFFFFF",
        "error": "#4E0002",
        "onError": "#FFFFFF",
        "errorContainer": "#8C0009",
        "onErrorContainer": "#FFFFFF",
        "background": "#F9F9FF",
        "onBackground": "#191C20",
        "surface": "#F9F9FF",
        "onSurface": "#000000",
        "surfaceVariant": "#E0E2EC",
        "onSurfaceVariant": "#21242B",
        "outline": "#40434A",
        "outlineVariant": "#40434A",
        "shadow": "#000000",
        "scrim": "#000000",
        "inverseSurface": "#2E3036",
        "inverseOnSurface": "#FFFFFF",
        "inversePrimary": "#E5ECFF",
        "primaryFixed": "#234373",
        "onPrimaryFixed": "#FFFFFF",
        "primaryFixedDim": "#032C5B",
        "onPrimaryFixedVariant": "#FFFFFF",
        "secondaryFixed": "#3A4354",
        "onSecondaryFixed": "#FFFFFF",
        "secondaryFixedDim": "#242D3D",
        "onSecondaryFixedVariant": "#FFFFFF",
        "tertiaryFixed": "#523A58",
        "onTertiaryFixed": "#FFFFFF",
        "tertiaryFixedDim": "#3B2441",
        "onTertiaryFixedVariant": "#FFFFFF",
        "surfaceDim": "#D9D9E0",
        "surfaceBright": "#F9F9FF",
        "surfaceContainerLowest": "#FFFFFF",
        "surfaceContainerLow": "#F3F3FA",
        "surfaceContainer": "#EDEDF4",
        "surfaceContainerHigh": "#E7E8EE",
        "surfaceContainerHighest": "#E2E2E9"
    }
}
