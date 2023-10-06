import rospy
from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class unsupported_message_job(JobInterface):

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_<message_type>
        """
        super().__init__(message, rfConnection, system, component)
        return

    def _doJob(self):
        rospy.logwarn(f"The message:\n{self.getMessage()}\nis not supported yet. Implement it in the Jobs modules!")
