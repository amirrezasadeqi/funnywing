import QtQuick 2.15

QtObject {
    readonly property ThemeBase theme: ThemeBase {
        Component.onCompleted: {
            // Corresponding properties to QML material module configurable properties
            themeMode = "Dark";
            primaryColor = "#FBFAFF";  // primary in m3 design
            accentColor = "#FFFFFF";  // does not have exact correspondence in m3 design
            foregroundColor = "#000000";  // onPrimary and ... in m3 material
            backgroundColor = "#FBFAFF";  // background of elements like buttos. equivalent to primary in m3 material
            // Inspired properties from m3 material design: m3.material.io
            m3["primary"] = "#FBFAFF";
            m3["surfaceTint"] = "#AAC7FF";
            m3["onPrimary"] = "#000000";
            m3["primaryContainer"] = "#B1CBFF";
            m3["onPrimaryContainer"] = "#000000";
            m3["secondary"] = "#FBFAFF";
            m3["onSecondary"] = "#000000";
            m3["secondaryContainer"] = "#C2CBE0";
            m3["onSecondaryContainer"] = "#000000";
            m3["tertiary"] = "#FFF9FA";
            m3["onTertiary"] = "#000000";
            m3["tertiaryContainer"] = "#E1C0E5";
            m3["onTertiaryContainer"] = "#000000";
            m3["error"] = "#FFF9F9";
            m3["onError"] = "#000000";
            m3["errorContainer"] = "#FFBAB1";
            m3["onErrorContainer"] = "#000000";
            m3["background"] = "#111318";
            m3["onBackground"] = "#E2E2E9";
            m3["surface"] = "#111318";
            m3["onSurface"] = "#FFFFFF";
            m3["surfaceVariant"] = "#44474E";
            m3["onSurfaceVariant"] = "#FBFAFF";
            m3["outline"] = "#C8CAD4";
            m3["outlineVariant"] = "#C8CAD4";
            m3["shadow"] = "#000000";
            m3["scrim"] = "#000000";
            m3["inverseSurface"] = "#E2E2E9";
            m3["inverseOnSurface"] = "#000000";
            m3["inversePrimary"] = "#002958";
            m3["primaryFixed"] = "#DDE7FF";
            m3["onPrimaryFixed"] = "#000000";
            m3["primaryFixedDim"] = "#B1CBFF";
            m3["onPrimaryFixedVariant"] = "#001634";
            m3["secondaryFixed"] = "#DEE7FD";
            m3["onSecondaryFixed"] = "#000000";
            m3["secondaryFixedDim"] = "#C2CBE0";
            m3["onSecondaryFixedVariant"] = "#0D1626";
            m3["tertiaryFixed"] = "#FCDDFF";
            m3["onTertiaryFixed"] = "#000000";
            m3["tertiaryFixedDim"] = "#E1C0E5";
            m3["onTertiaryFixedVariant"] = "#220E29";
            m3["surfaceDim"] = "#111318";
            m3["surfaceBright"] = "#37393E";
            m3["surfaceContainerLowest"] = "#0C0E13";
            m3["surfaceContainerLow"] = "#191C20";
            m3["surfaceContainer"] = "#1D2024";
            m3["surfaceContainerHigh"] = "#282A2F";
            m3["surfaceContainerHighest"] = "#33353A";
        }
    }
}
