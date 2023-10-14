from abc import ABC, abstractmethod


class FilterInterface(ABC):

    @abstractmethod
    def msgIsPassed(self, message) -> bool:
        """
        @return returns True if message passes the filter condition and False if it does not.
        """
        pass

    @abstractmethod
    def _readFilterConfig(self, configPath):
        pass