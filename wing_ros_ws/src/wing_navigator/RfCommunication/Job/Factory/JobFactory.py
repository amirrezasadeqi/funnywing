from RfCommunication.Job import Jobs as jobs


class JobFactory():
    def __init__(self):
        self._message = None
        return

    def setMessage(self, message):
        self._message = message
        return

    def getMessage(self):
        return self._message

    def createJob(self):
        if "heartbeat_job" == self._message:
            return jobs.heartbeat_job("heartbeat")
        else:
            return None
