# Copyright (c) 2020 
#Jiyo Jolly Palatti

"""
Package containing simple helper functions and classes 
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

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import tf
import rospy
import datetime
import math

from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaStatus

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    """
    Handle the info display
    """

    def __init__(self, role_name, width, height):
        self.role_name = role_name
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self._show_info = True
        self._info_text = []
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.tf_listener = tf.TransformListener()
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_status".format(self.role_name),
            CarlaEgoVehicleStatus, self.vehicle_status_updated)
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.vehicle_info_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_info".format(self.role_name),
            CarlaEgoVehicleInfo, self.vehicle_info_updated)
        self.latitude = 0
        self.longitude = 0
        self.manual_control = False
        self.gnss_subscriber = rospy.Subscriber(
            "/carla/{}/gnss/gnss1/fix".format(self.role_name), NavSatFix, self.gnss_updated)
        self.manual_control_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool, self.manual_control_override_updated)

        self.carla_status = CarlaStatus()
        self.status_subscriber = rospy.Subscriber(
            "/carla/status", CarlaStatus, self.carla_status_updated)

    def __del__(self):
        self.gnss_subscriber.unregister()
        self.vehicle_status_subscriber.unregister()
        self.vehicle_info_subscriber.unregister()

    def tick(self, clock):
        """
        tick method
        """
        self._notifications.tick(clock)

    def carla_status_updated(self, data):
        """
        Callback on carla status
        """
        self.carla_status = data
        self.update_info_text()

    def manual_control_override_updated(self, data):
        """
        Callback on vehicle status updates
        """
        self.manual_control = data.data
        self.update_info_text()

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on vehicle status updates
        """
        self.vehicle_status = vehicle_status
        self.update_info_text()

    def vehicle_info_updated(self, vehicle_info):
        """
        Callback on vehicle info updates
        """
        self.vehicle_info = vehicle_info
        self.update_info_text()

    def gnss_updated(self, data):
        """
        Callback on gnss position updates
        """
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.update_info_text()

    def update_info_text(self):
        """
        update the displayed info text
        """
        if not self._show_info:
            return
        try:
            (position, quaternion) = self.tf_listener.lookupTransform(
                '/map', self.role_name, rospy.Time())
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
            yaw = -math.degrees(yaw)
            x = position[0]
            y = -position[1]
            z = position[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x = 0
            y = 0
            z = 0
            yaw = 0
        heading = 'N' if abs(yaw) < 89.5 else ''
        heading += 'S' if abs(yaw) > 90.5 else ''
        heading += 'E' if 179.5 > yaw > 0.5 else ''
        heading += 'W' if -0.5 > yaw > -179.5 else ''
        fps = 0
        if self.carla_status.fixed_delta_seconds:
            fps = 1 / self.carla_status.fixed_delta_seconds
        self._info_text = [
            'Frame: % 22s' % self.carla_status.frame,
            'Simulation time: % 12s' % datetime.timedelta(
                seconds=int(rospy.get_rostime().to_sec())),
            'FPS: % 24.1f' % fps,
            '',
            'Vehicle: % 20s' % ' '.join(self.vehicle_info.type.title().split('.')[1:]),
            'Speed:   % 15.0f km/h' % (3.6 * self.vehicle_status.velocity),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (x, y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (self.latitude, self.longitude)),
            'Height:  % 18.0f m' % z,
            '']
        self._info_text += [
            ('Throttle:', self.vehicle_status.control.throttle, 0.0, 1.0),
            ('Steer:', self.vehicle_status.control.steer, -1.0, 1.0),
            ('Brake:', self.vehicle_status.control.brake, 0.0, 1.0),
            ('Reverse:', self.vehicle_status.control.reverse),
            ('Hand brake:', self.vehicle_status.control.hand_brake),
            ('Manual:', self.vehicle_status.control.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(self.vehicle_status.control.gear,
                                                      self.vehicle_status.control.gear),
            '']
        self._info_text += [('Manual ctrl:', self.manual_control)]
        if self.carla_status.synchronous_mode:
            self._info_text += [('Sync mode running:', self.carla_status.synchronous_mode_running)]
        self._info_text += ['', '', 'Press <H> for help']

    def toggle_info(self):
        """
        show/hide the info text
        """
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """
        display a notification for x seconds
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """
        display an error
        """
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """
        render the display
        """
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    """
    Support Class for info display, fade out text
    """

    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """
        set the text
        """
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, clock):
        """
        tick for fading
        """
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """
        render the fading
        """
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """
    Show the help text
    """

    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """
        Show/hide the help
        """
        self._render = not self._render

    def render(self, display):
        """
        render the help
        """
        if self._render:
            display.blit(self.surface, self.pos)