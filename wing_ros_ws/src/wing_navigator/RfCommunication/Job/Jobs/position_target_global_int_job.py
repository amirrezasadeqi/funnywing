import rospy
from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class position_target_global_int_job(JobInterface):

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_<message_type>
        """
        super().__init__(message, rfConnection, system, component)
        return

    def _doJob(self):
        rospy.loginfo(f"{self.getMessage()}")
