import rospy
from RfCommunication.Job.Interface.JobInterface import JobInterface


class heartbeat_job(JobInterface):

    def __init__(self, message):
        """

        @type message: MAVLink_<message_type>
        """
        super().__init__(message)
        return

    def _doJob(self):
        rospy.loginfo(f"{self.getMessage()}")
