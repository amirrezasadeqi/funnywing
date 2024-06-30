import QtQuick 2.15

QtObject {
    readonly property ThemeBase theme: ThemeBase {
        Component.onCompleted: {
            // Corresponding properties to QML material module configurable properties
            themeMode = "Light";
            primaryColor = "#00224A";  // primary in m3 design
            accentColor = "#FFFFFF";  // does not have exact correspondence in m3 design
            foregroundColor = "#FFFFFF";  // onPrimary and ... in m3 material
            backgroundColor = "#00224A";  // background of elements like buttos. equivalent to primary in m3 material
            // Inspired properties from m3 material design: m3.material.io
            m3["primary"] = "#00224A";
            m3["surfaceTint"] = "#405F91";
            m3["onPrimary"] = "#FFFFFF";
            m3["primaryContainer"] = "#234373";
            m3["onPrimaryContainer"] = "#FFFFFF";
            m3["secondary"] = "#192232";
            m3["onSecondary"] = "#FFFFFF";
            m3["secondaryContainer"] = "#3A4354";
            m3["onSecondaryContainer"] = "#FFFFFF";
            m3["tertiary"] = "#2F1A35";
            m3["onTertiary"] = "#FFFFFF";
            m3["tertiaryContainer"] = "#523A58";
            m3["onTertiaryContainer"] = "#FFFFFF";
            m3["error"] = "#4E0002";
            m3["onError"] = "#FFFFFF";
            m3["errorContainer"] = "#8C0009";
            m3["onErrorContainer"] = "#FFFFFF";
            m3["background"] = "#F9F9FF";
            m3["onBackground"] = "#191C20";
            m3["surface"] = "#F9F9FF";
            m3["onSurface"] = "#000000";
            m3["surfaceVariant"] = "#E0E2EC";
            m3["onSurfaceVariant"] = "#21242B";
            m3["outline"] = "#40434A";
            m3["outlineVariant"] = "#40434A";
            m3["shadow"] = "#000000";
            m3["scrim"] = "#000000";
            m3["inverseSurface"] = "#2E3036";
            m3["inverseOnSurface"] = "#FFFFFF";
            m3["inversePrimary"] = "#E5ECFF";
            m3["primaryFixed"] = "#234373";
            m3["onPrimaryFixed"] = "#FFFFFF";
            m3["primaryFixedDim"] = "#032C5B";
            m3["onPrimaryFixedVariant"] = "#FFFFFF";
            m3["secondaryFixed"] = "#3A4354";
            m3["onSecondaryFixed"] = "#FFFFFF";
            m3["secondaryFixedDim"] = "#242D3D";
            m3["onSecondaryFixedVariant"] = "#FFFFFF";
            m3["tertiaryFixed"] = "#523A58";
            m3["onTertiaryFixed"] = "#FFFFFF";
            m3["tertiaryFixedDim"] = "#3B2441";
            m3["onTertiaryFixedVariant"] = "#FFFFFF";
            m3["surfaceDim"] = "#D9D9E0";
            m3["surfaceBright"] = "#F9F9FF";
            m3["surfaceContainerLowest"] = "#FFFFFF";
            m3["surfaceContainerLow"] = "#F3F3FA";
            m3["surfaceContainer"] = "#EDEDF4";
            m3["surfaceContainerHigh"] = "#E7E8EE";
            m3["surfaceContainerHighest"] = "#E2E2E9";
        }
    }
}
