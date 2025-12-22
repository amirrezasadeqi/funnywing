import threading
import time
import rospy
from pymavlink import mavutil
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


# TODO: check if singleton pattern is suitable for this connection
class RfConnection(ConnectionInterface):
    def __init__(self, connectionString, baudRate, srcSystem, srcComponent, dialect, outBufWaitForMsg=1e-4):
        self._outBufWaitForMsg = outBufWaitForMsg
        self._inBuf = []
        self._outBuf = []
        self._connectionString = connectionString
        self._baudRate = baudRate
        self._srcSystem = srcSystem
        self._srcComponent = srcComponent
        self._dialect = dialect
        self._port = None

        self._startCommunication()

        return

    def read(self):
        if len(self._inBuf):
            return self._inBuf.pop(0)
        else:
            return None

    def write(self, message):
        self._outBuf.append(message)

    def getPort(self):
        return self._port

    def _startCommunication(self):
        self._initializePort()
        # Initialize threads for read/write and ... tasks
        self._recvThread = threading.Thread(target=self._recvLoop)
        self._sendThread = threading.Thread(target=self._sendLoop)
        self._recvThread.start()
        self._sendThread.start()
        return

    def _initializePort(self):
        # Initialize Mavlink serial port
        # TODO: Using mavlink 2.0 raises some CRC error, so commented below line.
        # os.environ["MAVLINK20"] = "1"
        try:
            if "/dev/tty" in self._connectionString:  # For serial port connections
                self._port = mavutil.mavlink_connection(self._connectionString, baud=self._baudRate,
                                                        source_system=self._srcSystem,
                                                        source_component=self._srcComponent,
                                                        dialect=self._dialect)
            else:  # For UDP/TCP connections, no baud rate is needed
                self._port = mavutil.mavlink_connection(self._connectionString, source_system=self._srcSystem,
                                                        source_component=self._srcComponent, dialect=self._dialect)
        except Exception:
            self._port = None
            print("The Connection is Not initialized Correctly!")
        return

    def _recvLoop(self):
        print("[RF] Starting MAVLink receive loop...", flush=True)
        while not rospy.is_shutdown():
            try:
                inMsg = self._port.recv_match(blocking=True, timeout=1.0)
                if inMsg:
                    msg_type = inMsg.get_type()
                    print(f"[RF] Received MAVLink message type: {msg_type}", flush=True)
                
                    if msg_type != "BAD_DATA":
                        if msg_type == "VFR_HUD":
                            print(f"[RF] VFR_HUD received! Airspeed={inMsg.airspeed:.2f}, Groundspeed={inMsg.groundspeed:.2f}, Throttle={inMsg.throttle}", flush=True)
                        self._inBuf.append(inMsg)
                    else:
                        print("[RF] MAVLink BAD_DATA received and ignored.", flush=True)
                else:
                    print("[RF] MAVLink receive timeout (no message).", flush=True)

            except Exception as e:
                print(f"[RF] Exception in _recvLoop: {e}", flush=True)
                time.sleep(0.5)
        return

    def _sendLoop(self):
        while not rospy.is_shutdown():
            if len(self._outBuf):
                outMsg = self._outBuf.pop(0)
                # if not a command and out of data -> continue
                try:
                    self._port.mav.send(outMsg)
                except Exception:
                    rospy.logwarn("Message coming from FCU by mavros, is in MAVLink_message base class or corrupted!")
            else:
                time.sleep(self._outBufWaitForMsg)
        return

    def __del__(self):
        # TODO: check if we need to join the running threads here or not.
        self._port.close()
