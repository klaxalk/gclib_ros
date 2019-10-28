#!/usr/bin/env python

import rospy

from gclib_python_wrapper.gclib_wrapper import GclibWrapper

from gclib_ros.srv import SingleAxisMotion,SingleAxisMotionResponse
from gclib_ros.srv import MultiAxisMotion,MultiAxisMotionResponse
from gclib_ros.msg import Position
from gclib_ros.srv import SetFloat,SetFloatResponse

from std_srvs.srv import Trigger,TriggerResponse

class Gclib:

    # #{ mainTimer()

    def mainTimer(self, event):

        rospy.loginfo_once('Main timer spinning')

        msg = Position()

        msg.stamp = rospy.Time.now()

        positions = []

        for i in range(0, self.n_stages):

            position = self.wrapper.getPositionUnit(i)
            positions.append(position)

            msg.position.append(position)

        self.publisher_position.publish(msg)

        rospy.loginfo_throttle(1.0, 'Position {}'.format(positions))

    # #} end of mainTimer()

    # #{ callbackGotoRelative()

    def callbackGotoRelative(self, req):

        rospy.loginfo('Goto relative axis {} value {}'.format(req.axis, req.value))

        res = self.wrapper.moveRelative(req.axis, req.value)

        if res:
            response = SingleAxisMotionResponse(True, "we are there")
            rospy.loginfo('Goto absolute finished')
        else:
            response = SingleAxisMotionResponse(True, "something went wrong")
            rospy.loginfo('Goto absolute finished')

        return response

    # #} end of callbackGotoRelative()

    # #{ callbackGotoSingle()

    def callbackGotoSingle(self, req):

        rospy.loginfo('Goto axis {} value {}'.format(req.axis, req.value))

        res = self.wrapper.moveAbsolute(req.axis, req.value)

        if res:
            response = SingleAxisMotionResponse(True, "we are there")
            rospy.loginfo('Goto absolute finished')
        else:
            response = SingleAxisMotionResponse(True, "something went wrong")
            rospy.logerr('Goto absolute failed')

        return response

    # #} end of callbackGotoSingle()

    # #{ callbackGotoRelativeX()

    def callbackGotoRelativeX(self, req):

        rospy.loginfo('Goto axis 0 value {}'.format(req.value))

        res = self.wrapper.moveRelative(0, req.value)

        if res:
            response = SetFloatResponse(True, "we are there")
            rospy.loginfo('Goto finished')
        else:
            response = SetFloatResponse(True, "something went wrong")
            rospy.logerr('Goto failed')

        return response

    # #} end of callbackGotoSingle()

    # #{ callbackGotoRelativeY()

    def callbackGotoRelativeY(self, req):

        rospy.loginfo('Goto axis 1 value {}'.format(req.value))

        res = self.wrapper.moveRelative(1, req.value)

        if res:
            response = SetFloatResponse(True, "we are there")
            rospy.loginfo('Goto finished')
        else:
            response = SetFloatResponse(True, "something went wrong")
            rospy.logerr('Goto failed')

        return response

    # #} end of callbackGotoSingle()

    # #{ callbackSetOrigin()

    def callbackSetOrigin(self, req):

        rospy.loginfo('reseting origin')

        res = self.wrapper.setOrigin()

        if res:
            response = TriggerResponse(True, "origin set")
            rospy.loginfo('origin set')
        else:
            response = TriggerResponse(True, "origin not set")
            rospy.logerr('origin not set')

        return response

    # #} end of callbackGotoSingle()

    # #{ callbackGotoAll()

    def callbackGotoAll(self, req):

        rospy.loginfo('Goto values [{}]'.format(req.values))

        res = self.wrapper.moveAllAbsolute(req.values)

        if res:
            response = MultiAxisMotionResponse(True, "we are there")
            rospy.loginfo('Goto all finished')
        else:
            response = MultiAxisMotionResponse(True, "something went wrong")
            rospy.logerr('Goto all failed')

        return response

    # #} end of callbackGotoAll()

    # #{ __init__()

    def __init__(self):

        rospy.init_node('gclib', anonymous=True)

        # parameters
        address = rospy.get_param('~address')
        publisher_rate = rospy.get_param('~publisher_rate')
        dummy = rospy.get_param('~dummy')
        debug = rospy.get_param('~debug')

        self.n_stages = rospy.get_param('~n_stages')

        steps2unit = rospy.get_param('~steps2unit')

        speed = rospy.get_param('~speed')
        acceleration = rospy.get_param('~acceleration')
        deceleration = rospy.get_param('~deceleration')

        forward_limit = rospy.get_param('~forward_limit')
        backward_limit = rospy.get_param('~backward_limit')

        self.wrapper = GclibWrapper(2, steps2unit, dummy, debug)

        self.wrapper.open(address)

        self.wrapper.setOrigin()

        self.wrapper.setSpeed(speed)
        self.wrapper.setAcceleration(acceleration)
        self.wrapper.setDeceleration(deceleration)

        self.wrapper.setForwardLimit(forward_limit)
        self.wrapper.setBackwardLimit(backward_limit)

        self.wrapper.motorsOn()

        rospy.Service("~goto_relative", SingleAxisMotion, self.callbackGotoRelative)
        rospy.Service("~goto_absolute", SingleAxisMotion, self.callbackGotoSingle)
        rospy.Service("~goto", MultiAxisMotion, self.callbackGotoAll)
        rospy.Service("~goto_relative_x", SetFloat, self.callbackGotoRelativeX)
        rospy.Service("~goto_relative_y", SetFloat, self.callbackGotoRelativeY)
        rospy.Service("~set_origin", Trigger, self.callbackSetOrigin)

        self.publisher_position = rospy.Publisher("~position", Position, queue_size=1)

        rospy.Timer(rospy.Duration(1.0/publisher_rate), self.mainTimer)

        rospy.spin()

    # #} end of __init__()

if __name__ == '__main__':
    try:
        gclib = Gclib()
    except rospy.ROSInterruptException:
        pass
