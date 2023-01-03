# In this file we create tempelate dictionaries for navigation commands
# transferring over RF link.

class navigation_commands:
    @staticmethod
    def active_mode(flight_mode):
        return {
            "command_type": "active_mode",
            "flight_mode": flight_mode
        }

    @staticmethod
    def arm_takeoff():
        return {
            "command_type": "arm_takeoff"
        }
