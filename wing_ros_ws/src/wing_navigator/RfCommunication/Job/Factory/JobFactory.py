from RfCommunication.Job import Jobs as jobs


class JobFactory(object):
    def __init__(self):
        self._message = None
        return

    def setMessage(self, message):
        """
        @param message: This is a mavlink message. Not the base class
         MAVLink_message but its child classes(e.g. MAVLink_heartbeat_message)
         . since we need to pass it to job class and use the attributes of those
         child classes in there.
        """
        self._message = message
        return

    def createJob(self):
        jobType = self._getJobType()
        job = getattr(jobs, jobType)(self._message)
        return job

    def _getJobType(self):
        msgType: str = self._message.get_type()
        jobType = msgType.lower() + "_job"
        return jobType
