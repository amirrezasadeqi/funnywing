from RfCommunication.Job import Jobs as jobs
from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.MavrosPublishManager.MavrosPublishManager import MavrosPublishManager
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class JobFactory(object):
    def __init__(self, rfConnection: ConnectionInterface, mavrosPublisherManager: MavrosPublishManager, system,
                 component):
        self._rfConnection = rfConnection
        self._mavrosPublisherManager = mavrosPublisherManager
        self._system = system
        self._component = component
        self._job: JobInterface = jobs.unsupported_message_job(None, self._rfConnection,
                                                               self._mavrosPublisherManager.getPublishers(
                                                                   "UNSUPPORTED_MESSAGE"), self._system,
                                                               self._component)
        return

    def createJob(self, message) -> JobInterface:
        """
        @param message: MAVLink_<message_type> coming from RF interface
        @return: JobInterface to be used in User classes
        """
        jobType = self._getJobType(message.get_type())

        try:
            self._job = getattr(jobs, jobType)(message, self._rfConnection,
                                               self._mavrosPublisherManager.getPublishers(message.get_type()),
                                               self._system, self._component)
        except Exception:
            self._job = jobs.unsupported_message_job(message, self._rfConnection,
                                                     self._mavrosPublisherManager.getPublishers(
                                                         "UNSUPPORTED_MESSAGE"), self._system, self._component)

        return self._job

    def _getJobType(self, messageType: str):
        return messageType.lower() + "_job"
