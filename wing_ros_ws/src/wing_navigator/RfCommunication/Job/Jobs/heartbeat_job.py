import rospy
from RfCommunication.Job.Interface.JobInterface import JobInterface


class heartbeat_job(JobInterface):

    def __init__(self, message):
        self._message = message

    def getMessage(self):
        return self._message

    def _doJob(self):
        rospy.loginfo(f"{self._message}")
