from RfCommunication.Job import Jobs as jobs
from RfCommunication.Job.Interface.JobInterface import JobInterface


class JobFactory(object):
    def __init__(self):
        self._job: JobInterface = jobs.unsupported_message_job(None)
        return

    def createJob(self, message) -> JobInterface:
        """
        @param message: MAVLink_<message_type> coming from RF interface
        @return: JobInterface to be used in User classes
        """
        jobType = self._getJobType(message.get_type())

        try:
            self._job = getattr(jobs, jobType)(message)
        except Exception:
            self._job = jobs.unsupported_message_job(message)

        return self._job

    def _getJobType(self, messageType: str):
        return messageType.lower() + "_job"
