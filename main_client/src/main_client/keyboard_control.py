# Copyright (c) 2020 
#Jiyo Jolly Palatti

# Allows controlling a vehicle with a keyboard. Adapted from CARLA Manual Control Example 

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
from std_msgs.msg import Bool
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaStatus
import rospy



try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_b
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """
    Handle input events
    """

    def __init__(self, role_name, hud):
        self.role_name = role_name
        self.hud = hud

        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0

        self.vehicle_control_manual_override_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool, queue_size=1, latch=True)
        self.reset_scenario_publisher = rospy.Publisher(
            "/carla/reset_scenario", Bool, queue_size=1, latch=False)
        self.vehicle_control_manual_override = False
        self.auto_pilot_enable_publisher = rospy.Publisher(
            "/carla/{}/enable_autopilot".format(self.role_name), Bool, queue_size=1)
        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd_manual".format(self.role_name),
            CarlaEgoVehicleControl, queue_size=1)
        self.carla_status_subscriber = rospy.Subscriber(
            "/carla/status", CarlaStatus, self._on_new_carla_frame)

        self.set_autopilot(self._autopilot_enabled)

        self.set_vehicle_control_manual_override(
            self.vehicle_control_manual_override)  # disable manual override

    def __del__(self):
        self.auto_pilot_enable_publisher.unregister()
        self.vehicle_control_publisher.unregister()
        self.vehicle_control_manual_override_publisher.unregister()

    def set_vehicle_control_manual_override(self, enable):
        """
        Set the manual control override
        """
        self.hud.notification('Set vehicle control manual override to: {}'.format(enable))
        self.vehicle_control_manual_override_publisher.publish((Bool(data=enable)))

    def set_reset_scenario(self, enable):
        """
        Set the manual control override
        """
        self.hud.notification('Resetting scenario....')
        self.reset_scenario_publisher.publish((Bool(data=enable)))


    def set_autopilot(self, enable):
        """
        enable/disable the autopilot
        """
        self.auto_pilot_enable_publisher.publish(Bool(data=enable))

    # pylint: disable=too-many-branches
    def parse_events(self, clock):
        """
        parse an input event
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_F1:
                    self.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and
                                          pygame.key.get_mods() & KMOD_SHIFT):
                    self.hud.help.toggle()
                elif event.key == K_b:
                    self.vehicle_control_manual_override = not self.vehicle_control_manual_override
                    self.set_vehicle_control_manual_override(self.vehicle_control_manual_override)
                elif event.key == K_BACKSPACE:
                    self.set_reset_scenario(True)
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self.hud.notification('%s Transmission' % (
                        'Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self.set_autopilot(self._autopilot_enabled)
                    self.hud.notification('Autopilot %s' % (
                        'On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled and self.vehicle_control_manual_override:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0

    def _on_new_carla_frame(self, data):
        """
        callback on new frame

        As CARLA only processes one vehicle control command per tick,
        send the current from within here (once per frame)
        """
        if not self._autopilot_enabled and self.vehicle_control_manual_override:
            try:
                self.vehicle_control_publisher.publish(self._control)
            except ROSException as error:
                rospy.logwarn("Could not send vehicle control: {}".format(error))

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        parse key events
        """
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
