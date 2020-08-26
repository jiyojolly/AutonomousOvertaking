#!/usr/bin/env python
#
# Copyright (c) 2020 
# Jiyo Jolly Palatti
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""

"""

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import sys
import ast
import carla
import rospy
import random
from carla_msgs.msg import CarlaWorldInfo

class NPCVehicles(object):
    """Class for instantiating other non ego vehicles of different scenarios"""
    def __init__(self):
        rospy.init_node('ego_vehicle', anonymous=True)
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self.timeout = rospy.get_param('/carla/timeout', '2')

        #Get world & settings
        self.client = None
        self.world = None
        self.settings = None
        self._actor_filter = 'vehicle.*'
 
        self.npc_agents = []
        self.spawn_points = [] 

        

    def get_spawnpoint(self,loc):
        return carla.Transform(carla.Location(x=loc[0],y=loc[1],z=loc[2]),carla.Rotation(yaw=loc[3]))


    def init_npcs(self):

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        # spawn_locs = [[-198,-85,5,-90], [-198,-95,5,-90], [-198,-105,5,-90]]
        spawn_locs = ast.literal_eval(rospy.get_param('/npc_vehicles/spawn_locs', '[[-198,-85,5,-90], [-198,-95,5,-90], [-198,-105,5,-90]]'))
        self.spawn_points = [self.get_spawnpoint(x) for x in spawn_locs]

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
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, False)))

        for response in self.client.apply_batch_sync(batch, self.settings.synchronous_mode):
            if response.error:
                logging.error(response.error)
            else:
                self.npc_agents.append(response.actor_id)
        rospy.loginfo("Spawned subject vehicles")

    def destroy(self):
        """
        destroy the current all subject vehicles
        """
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_agents])
        self.npc_agents = []
        rospy.loginfo("Destroyed all subject vehicles")


    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for world info!")
            sys.exit(1)

        rospy.loginfo("CARLA world available. Spawn subject vehicles...")

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        
        self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        self.init_npcs()
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        #Initialize subject vehicles
        
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    npcs = NPCVehicles()
    try:
        npcs.run()
    finally:
        if npcs is not None:
            npcs.destroy()


if __name__ == '__main__':
    main()

