from abc import ABC, abstractmethod


class ConnectionInterface(ABC):
    @abstractmethod
    def read(self):
        pass

    @abstractmethod
    def write(self, message):
        pass
