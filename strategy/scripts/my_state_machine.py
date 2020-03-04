import json
import math
import warnings
from statemachine import StateMachine, State
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from strategy.cfg import RobotConfig
from my_ros_bridge.my_ros_bridge import Robot
from mir_bridge.mir_bridge import MIR

#HOST = "http://192.168.50.220:8080/v2.0.0"
HOST = "http://192.168.12.20:8080/v2.0.0"

class MyStateMachine(Robot, StateMachine):

    def __init__(self, sim=False):
        super(MyStateMachine, self).__init__(sim)
        StateMachine.__init__(self)
        dsrv = DynamicReconfigureServer(RobotConfig, self.callback)
        self.dclient = DynamicReconfigureClient(
                       "core", timeout=30, config_callback=None)
        self.mir = MIR(HOST)

    def callback(self, config, level):
        self.start = config['start']
        self.go_home = config['go_home']

        return config

    idle = State('Idle', initial=True)
    home = State('Home')
    move = State('Move')
    arm = State('Arm')

    toIdle = move.to(idle) | home.to(idle) | arm.to(idle)
    toHome = idle.to(home) | move.to(home) | home.to.itself() | arm.to(home)
    toMove = idle.to(move) | home.to(move) | move.to.itself() | arm.to(move)
    toArm = move.to(arm) | idle.to(arm)

    def on_toIdle(self):
        print("to IDLE, change MiR to 'Pause'")
        self.mir.set_status("Pause")
        self.mir.clear_mission_queue()
        self.dclient.update_configuration({"start": False})
        self.dclient.update_configuration({"go_home": False})

    def on_toHome(self):
        self.toMove("TKU_ToHOME")

    def on_toMove(self, mission):
        self.mir.set_status("Ready")
        print("to Move with mission {}".format(mission))

        guid = self.mir.get_mission_guid(mission)
        if guid is None:
            warnings.warn("[WARRING] No this position name!!!!!")
        else:
            self.mir.add_mission_to_queue(guid.encode('utf-8'))

    def on_toArm(self, mission):
        print("Call Arm")
        self.call_arm(mission)
