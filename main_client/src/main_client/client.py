#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Welcome to CARLA ROS manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    B            : toggle manual control

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

import datetime
import math
import numpy

import rospy
import tf
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaStatus

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

from main_client.ui_utils import HUD
from main_client.keyboard_control import KeyboardControl

import sys
print('\n'.join(sys.path))
# ==============================================================================
# -- Render Class ---------------------------------------------------------------------
# ==============================================================================


class RenderHandle(object):
    """
    Handle the rendering
    """

    def __init__(self, role_name, hud):
        self._surface = None
        self.hud = hud
        self.role_name = role_name
        self.image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/view/image_color".format(self.role_name),
            Image, self.on_view_image)
        self.collision_subscriber = rospy.Subscriber(
            "/carla/{}/collision".format(self.role_name), CarlaCollisionEvent, self.on_collision)
        self.lane_invasion_subscriber = rospy.Subscriber(
            "/carla/{}/lane_invasion".format(self.role_name),
            CarlaLaneInvasionEvent, self.on_lane_invasion)

    def on_collision(self, data):
        """
        Callback on collision event
        """
        intensity = math.sqrt(data.normal_impulse.x**2 +
                              data.normal_impulse.y**2 + data.normal_impulse.z**2)
        self.hud.notification('Collision with {} (impulse {})'.format(
            data.other_actor_id, intensity))

    def on_lane_invasion(self, data):
        """
        Callback on lane invasion event
        """
        text = []
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_OTHER:
                text.append("Other")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_BROKEN:
                text.append("Broken")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                text.append("Solid")
            else:
                text.append("Unknown ")
        self.hud.notification('Crossed line %s' % ' and '.join(text))

    def on_view_image(self, image):
        """
        Callback when receiving a camera image
        """
        array = numpy.frombuffer(image.data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        """
        render the current image
        """
        if self._surface is not None:
            display.blit(self._surface, (0, 0))
        self.hud.render(display)

    def destroy(self):
        """
        destroy all objects
        """
        self.image_subscriber.unregister()
        self.collision_subscriber.unregister()
        self.lane_invasion_subscriber.unregister()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node('carla_manual_control', anonymous=True)

    role_name = rospy.get_param("~role_name", "ego_vehicle")

    # resolution should be similar to spawned camera with role-name 'view'
    resolution = {"width": 800, "height": 600}

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("Main Client")
    render_obj = None
    try:
        display = pygame.display.set_mode(
            (resolution['width'], resolution['height']),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(role_name, resolution['width'], resolution['height'])
        render_obj = RenderHandle(role_name, hud)
        controller = KeyboardControl(role_name, hud)

        clock = pygame.time.Clock()

        while not rospy.core.is_shutdown():
            clock.tick_busy_loop(60)
            if controller.parse_events(clock):
                return
            hud.tick(clock)
            render_obj.render(display)
            pygame.display.flip()

    finally:
        if render_obj is not None:
            render_obj.destroy()
        pygame.quit()


if __name__ == '__main__':

    main()
