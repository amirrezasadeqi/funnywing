from abc import ABC, abstractmethod


class JobInterface(ABC):

    def runJob(self):
        self._doJob()
        return

    @abstractmethod
    def getMessage(self):
        pass

    @abstractmethod
    def _doJob(self):
        pass
