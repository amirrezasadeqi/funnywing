import rospy
from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class home_position_job(JobInterface):

    def __init__(self, message, rfConnection: ConnectionInterface, mavrosPublishers, system, component):
        """

        @type message: MAVLink_<message_type>
        """
        super().__init__(message, rfConnection, mavrosPublishers, system, component)
        return

    def _doJob(self):
        rospy.loginfo(f"{self.getMessage()}")
