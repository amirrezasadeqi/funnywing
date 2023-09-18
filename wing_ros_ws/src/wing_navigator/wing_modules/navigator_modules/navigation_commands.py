# In this file we create tempelate dictionaries for navigation commands
# transferring over RF link.

class navigation_commands:
    @staticmethod
    def active_mode(flight_mode, flight_mode_number):
        return {
            "command_type": "active_mode",
            "flight_mode": flight_mode,
            "flight_mode_number": flight_mode_number
        }

    @staticmethod
    def arm_takeoff():
        return {
            "command_type": "arm_takeoff"
        }

    @staticmethod
    def simple_goto(lat, lon, alt):
        return {
            "command_type": "simple_goto",
            "lat": lat,
            "lon": lon,
            "alt": alt
        }
