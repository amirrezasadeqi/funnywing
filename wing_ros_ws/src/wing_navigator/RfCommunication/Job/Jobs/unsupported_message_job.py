import rospy
from RfCommunication.Job.Interface.JobInterface import JobInterface


class unsupported_message_job(JobInterface):

    def __init__(self, message):
        """

        @type message: MAVLink_<message_type>
        """
        super().__init__(message)
        return

    def _doJob(self):
        rospy.logwarn(f"The message:\n{self.getMessage()}\nis not supported yet. Implement it in the Jobs modules!")
