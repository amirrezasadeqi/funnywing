from abc import ABC, abstractmethod


class JobInterface(ABC):

    def __init__(self, message):
        """
        @param message: This is a mavlink message. Not the base class
         MAVLink_message but its child classes(e.g. MAVLink_heartbeat_message)
         . since we need to pass it to job class and use the attributes of those
         child classes in there.
        """
        self._message = None
        self.setMessage(message)
        return

    def runJob(self):
        self._doJob()
        return

    def setMessage(self, message):
        self._message = message
        return

    def getMessage(self):
        return self._message

    @abstractmethod
    def _doJob(self):
        pass
