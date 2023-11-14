from pygeodesy.geoids import GeoidPGM


class EllipsoidMSLConversion(object):
    def __init__(self):
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        return

    def mslToEllipsoid(self, msl_pos):
        wgs_pos = [msl_pos[0], msl_pos[1], msl_pos[2] + self._geoid_height(msl_pos[0], msl_pos[1])]
        return wgs_pos

    def EllipsoidToMsl(self, wgs_pos):
        msl_pos = [wgs_pos[0], wgs_pos[1], wgs_pos[2] - self._geoid_height(wgs_pos[0], wgs_pos[1])]
        return msl_pos

    def _geoid_height(self, lat, lon):
        """Calculates AMSL to ellipsoid conversion offset.
        Uses EGM96 data with 5' grid and cubic interpolation.
        The value returned can help you convert from meters
        above mean sea level (AMSL) to meters above
        the WGS84 ellipsoid.

        If you want to go from AMSL to ellipsoid height, add the value.

        To go from ellipsoid height to AMSL, subtract this value.
        """
        return self._egm96.height(lat, lon)
