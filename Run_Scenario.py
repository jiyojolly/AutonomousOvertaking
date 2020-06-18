#!/usr/bin/env python

# Copyright (c) 2020 
#Jiyo Jolly Palatti


"""
Welcome to Autonomous Scenario Client.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down

    Backspace    : Reset Scenario

    C            : change weather (Shift+C reverse)

    ESC          : quit
    

"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla

from carla import ColorConverter as cc
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error

import argparse
import logging

try:
    import pygame
    
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# import user functions
from KeyboardControl import KeyboardControl
from World_Player import World
from Utils import HUD


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def main_loop(args):
    pygame.init()
    pygame.font.init()
    world = None 

    try:
        client = carla.Client(args.host, args.port)

        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client, hud, args)

        
        ## Control Section
        # Keyboard control
        controller = KeyboardControl(world, args.autopilot)

        # Agent control
        agent = BehaviorAgent(world.player, behavior='normal')

        spawn_points = world.map.get_spawn_points()
        random.shuffle(spawn_points)

        if spawn_points[0].location != agent.vehicle.get_location():
            destination = spawn_points[0].location
        else:
            destination = spawn_points[1].location

        agent.set_destination(agent.vehicle.get_location(), destination, clean=True)


        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(30)
            if controller.parse_events(client, world, clock):
                print("Stopping Client!! Destroying actors")
                return

            # # As soon as the server is ready continue!
            # if not world.world.wait_for_tick(10.0):
            #     continue

            agent.update_information(world)
            
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

            speed_limit = world.player.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)

            control = agent.run_step()
            world.player.apply_control(control)


    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy(client)

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--deltatime',
        default=0.05,
        type=float,
        help='Fixed delta seconds (default: 0.05)')
    argparser.add_argument(
        '--syncmode',
        default=False,
        action='store_true',
        help='Enable Synchronous Mode (default: False)')
    argparser.add_argument(
        '--map',
        metavar='MAP',
        default='Town07',
        help='Fixed delta seconds (default: Town07)')

    
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        main_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
