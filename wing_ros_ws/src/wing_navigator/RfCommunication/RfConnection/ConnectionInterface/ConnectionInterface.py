from abc import ABC, abstractmethod


class ConnectionInterface(ABC):
    @abstractmethod
    def read(self):
        pass

    @abstractmethod
    def write(self, message):
        pass

    @abstractmethod
    def getPort(self):
        """
        Gives the pymavlink.mavutil.mavlink_connection object.
        """
        pass
