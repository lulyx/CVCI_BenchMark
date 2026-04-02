import py_trees
import carla
import sys
import math

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (Criterion, 
    BrokenDownVehicleBrakeCriterion, BrokenDownVehicleBypassCriterion, BrokenDownVehicleResumeCriterion)

def ALERT(msg):
    sys.stderr.write(f"\n\033[91m [SCENARIO_ALERT] {msg} \033[0m\n")
    sys.stderr.flush()

# --- 行为：维持自车速度（增加终点退出逻辑） ---
class MaintainEgoVelocity(AtomicBehavior):
    def __init__(self, ego_vehicle, target_speed, goal_location, name="MaintainEgoVelocity"):
        super(MaintainEgoVelocity, self).__init__(name)
        self._ego_vehicle = ego_vehicle
        self._target_speed = target_speed 
        self._goal_location = goal_location
        self._has_been_controlled = False 

    def update(self):
        if self._ego_vehicle is None or not self._ego_vehicle.is_alive:
            return py_trees.common.Status.FAILURE
        
        # 【核心修改】检查是否到达终点，如果到了就返回 SUCCESS 允许 Parallel 节点结束
        ego_loc = self._ego_vehicle.get_location()
        dist_to_goal = ego_loc.distance(self._goal_location)
        if dist_to_goal < 5.0:
            return py_trees.common.Status.SUCCESS

        if self._has_been_controlled:
            return py_trees.common.Status.RUNNING

        control = self._ego_vehicle.get_control()
        v = self._ego_vehicle.get_velocity()
        current_speed = math.sqrt(v.x**2 + v.y**2)

        if current_speed > 2.0 and (abs(control.throttle) > 0.2 or abs(control.brake) > 0.2):
            ALERT("Manual Control Detected! Speed Lock Released.")
            self._has_been_controlled = True
            return py_trees.common.Status.RUNNING

        self._ego_vehicle.set_target_velocity(carla.Vector3D(x=self._target_speed, y=v.y, z=v.z))
        return py_trees.common.Status.RUNNING

# --- 行为：NPC 与自车同步移动（增加终点退出逻辑） ---
class YAxisPositionSync(AtomicBehavior):
    def __init__(self, actor, ego_vehicle, goal_location, name="YAxisPositionSync"):
        super(YAxisPositionSync, self).__init__(name)
        self._actor = actor
        self._ego_vehicle = ego_vehicle
        self._goal_location = goal_location
        self._x_offset = None

    def update(self):
        if self._actor is None or not self._actor.is_alive:
            return py_trees.common.Status.FAILURE
        
        # 【核心修改】自车到终点后，同步行为也返回 SUCCESS
        ego_trans = self._ego_vehicle.get_transform()
        if ego_trans.location.distance(self._goal_location) < 5.0:
            return py_trees.common.Status.SUCCESS

        npc_trans = self._actor.get_transform()

        if self._x_offset is None:
            self._x_offset = npc_trans.location.x - ego_trans.location.x
            self.initial_y = npc_trans.location.y
            self.initial_z = npc_trans.location.z
            self._actor.set_simulate_physics(False)
            ALERT(f"Sync Linked for NPC {self._actor.id}")
            return py_trees.common.Status.RUNNING

        new_loc = carla.Location(x=ego_trans.location.x + self._x_offset, y=self.initial_y, z=self.initial_z)
        self._actor.set_transform(carla.Transform(new_loc, npc_trans.rotation))
        return py_trees.common.Status.RUNNING

# --- 主场景类 ---
class BrokenDownVehicle(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        ALERT("BrokenDownVehicle Scenario Initializing...")
        self._xml_speed = config.ego_vehicles[0].speed
        # 统一目标点
        self.goal_location = carla.Location(x=-152.7, y=-12.0, z=0.5) 
        super(BrokenDownVehicle, self).__init__("BrokenDownVehicle", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        ALERT(f"Spawning {len(config.other_actors)} NPC Actors")
        for actor_conf in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(actor_conf.model, actor_conf.transform)
            if actor:
                actor.set_simulate_physics(True)
                self.other_actors.append(actor)

    def _create_behavior(self):
        ALERT("Building Behavior Tree")
        # --- 策略改为 SUCCESS_ON_ALL ---
        root = py_trees.composites.Parallel("ScenarioRoot", 
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        ego = self.ego_vehicles[0]

        target_v = -self._xml_speed if self._xml_speed > 0 else -10.0
        
        # 1. 控速行为（需传入目标点以实现退出）
        root.add_child(MaintainEgoVelocity(ego, target_speed=target_v, goal_location=self.goal_location))

        # 2. 同步行为（需传入目标点以实现退出）
        for i in range(min(3, len(self.other_actors))):
            if self.other_actors[i]:
                root.add_child(YAxisPositionSync(self.other_actors[i], ego, goal_location=self.goal_location))

        # 3. 目的地触发成功
        root.add_child(InTriggerDistanceToLocation(ego, self.goal_location, distance=3.0))
        
        return root

    def _create_test_criteria(self):
        ALERT("Generating Assessment Criteria")
        criteria = []
        ego = self.ego_vehicles[0]
        hazard = self.other_actors[4] if len(self.other_actors) > 4 else None
        
        criteria.append(BrokenDownVehicleBrakeCriterion(ego, hazard))
        if hazard:
            criteria.append(BrokenDownVehicleBypassCriterion(ego, hazard))
        criteria.append(BrokenDownVehicleResumeCriterion(ego, self.goal_location))
        return criteria