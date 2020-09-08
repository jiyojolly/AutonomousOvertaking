#!/usr/bin/env python

# Copyright (c) 2020 
#Jiyo Jolly Palatti


"""
Definitons for World & Client.

"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

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
import weakref
import collections
import random
import math


try:
    import pygame
    
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

#Import User defined
import Utils
from Utils import find_weather_presets, get_actor_display_name, CameraManager 


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, client, hud, args):
        self.world = client.load_world(args.map)
        # self.world = client.get_world()
        self.settings = self.world.get_settings()
        self.settings.fixed_delta_seconds = args.deltatime
        print("Synchronous Mode "+ str(args.syncmode))
        self.settings.synchronous_mode = args.syncmode
        self.world.apply_settings(self.settings)

        self.actor_role_name = args.rolename
        
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        #UI Elements
        self.hud = hud

        self.player = None
        #Create Sensor Objects
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.camera_manager = None

        #Weather 
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma

        #Other Agents
        self.npc_agents = []
        self.spawn_points = []  

        #Initialise World
        self.init(client)      

        #Recording
        self.recording_enabled = False
        self.recording_start = 0

    def init(self,client):

        #Register callback functions on world tick
        self.world.on_tick(self.hud.on_world_tick)
        
        #Setup Scenario
        # Setup Ego Vehicle
        self.init_ego()

        # Set up the sensors.
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0 
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        # Setup npcs Vehicle
        self.init_npcs(client)

    def restart(self, client):
        self.destroy(client)
        print("Resetting Scenario....")
        self.init(client)


    def init_ego(self):
        blueprint = self.world.get_blueprint_library().find("vehicle.audi.a2")
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)

        # # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # while self.player is None:
        #     if not self.map.get_spawn_points():
        #         print('There are no spawn points available in your map/town.')
        #         print('Please add some Vehicle Spawn Point to your UE4 scene.')
        #         sys.exit(1)
        #     spawn_points = self.map.get_spawn_points()
        #     spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        #     self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_point = Utils.get_spawnpoint([-198,-75,5,-90])
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

    def init_npcs(self,client):

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        spawn_locs = [[-198,-85,5,-90], [-198,-95,5,-90], [-198,-105,5,-90]]
        self.spawn_points = [Utils.get_spawnpoint(x) for x in spawn_locs]

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(self.spawn_points):    
            blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'NPC_'+str(n))
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        for response in client.apply_batch_sync(batch, self.settings.synchronous_mode):
            if response.error:
                logging.error(response.error)
            else:
                self.npc_agents.append(response.actor_id)
      

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)


    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self,client):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.player]
        
        for actor in actors:
            if actor is not None:
                actor.destroy()      

        client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_agents])
        self.npc_agents = []
        self.camera_manager.sensor = None
        self.collision_sensor.sensor = None
        self.lane_invasion_sensor.sensor = None
        self.player = None 

        # [print(x.type_id) for x in actors]          
        # [print((self.world.get_actor(x)).type_id) for x in self.npc_agents]
        
        print("All actors destroyed!!!")



# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))
        # print("Collision Sensor spwaned and listening")

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        # print("Collision")
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))