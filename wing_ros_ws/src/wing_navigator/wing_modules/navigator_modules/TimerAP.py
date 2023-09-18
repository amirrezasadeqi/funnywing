#!/usr/bin/env python

import rospy
import threading
import time


class TimerEvent(object):
    """
    Constructor.
    @param last_expected: in a perfect world, this is when the previous callback should have happened
    @type  last_expected: rospy.Time
    @param last_real: when the callback actually happened
    @type  last_real: rospy.Time
    @param current_expected: in a perfect world, this is when the current callback should have been called
    @type  current_expected: rospy.Time
    @param last_duration: contains the duration of the last callback (end time minus start time) in seconds.
                          Note that this is always in wall-clock time.
    @type  last_duration: float
    """

    def __init__(self, last_expected, last_real, current_expected, current_real, last_duration):
        self.last_expected = last_expected
        self.last_real = last_real
        self.current_expected = current_expected
        self.current_real = current_real
        self.last_duration = last_duration


class TimerAP(rospy.Timer):
    def __init__(self, period, callback, cb_arg_list, oneshot=False):
        """
        Constructor.
        @param period: desired period between callbacks
        @type  period: rospy.Time
        @param callback: callback to be called
        @type  callback: function taking rospy.TimerEvent
        @param oneshot: if True, fire only once, otherwise fire continuously until shutdown is called [default: False]
        @type  oneshot: bool
        """
        threading.Thread.__init__(self)
        self._period = period
        self._callback = callback
        # Our customization
        # Adding self._cb_arg_list to parameter list to be able to pass arguments to callback function
        self._cb_arg_list = cb_arg_list
        self._oneshot = oneshot
        self._shutdown = False
        self.setDaemon(True)
        self.start()

    def run(self):
        r = rospy.Rate(1.0 / self._period.to_sec())
        current_expected = rospy.rostime.get_rostime() + self._period
        last_expected, last_real, last_duration = None, None, None
        while not rospy.core.is_shutdown() and not self._shutdown:
            r.sleep()
            current_real = rospy.rostime.get_rostime()
            start = time.time()
            # Our customization
            # Adding self._cb_arg_list to parameter list to be able to pass arguments to callback function
            self._callback(self._cb_arg_list, (last_expected, last_real,
                           current_expected, current_real, last_duration))
            if self._oneshot:
                break
            last_duration = time.time() - start
            last_expected, last_real = current_expected, current_real
            current_expected += self._period
