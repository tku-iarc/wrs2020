import json
from statemachine import StateMachine, State
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from strategy.cfg import RobotConfig
from my_ros_bridge.my_ros_bridge import Robot
from mir_bridge.mir_bridge import MIR

HOST = "http://192.168.50.220:8080/v2.0.0"

class MyStateMachine(Robot, StateMachine):

    def __init__(self, sim=False):
        super(MyStateMachine, self).__init__(sim)
        StateMachine.__init__(self)
        dsrv = DynamicReconfigureServer(RobotConfig, self.callback)
        self.mir = MIR(HOST)

    def callback(self, config, level):
        self.start = config['start']
        self.go_home = config['go_home']

        return config

    idle = State('Idle', initial=True)
    move = State('Move')

    toIdle = move.to(idle) | idle.to.itself()
    toMove = idle.to(move) | move.to.itself()

    def on_toIdle(self):
        print("to IDLE, change MiR to 'Pause'")
        self.mir.status("Pause")
        self.mir.clear_mission_queue()

    def on_toMove(self, mission):
        self.mir.status("Ready")
        print("to Move with mission {}".format(mission))

        guid = self.mir.get_mission_guid(mission)
        if guid is None:
            print("[WARRING] No this position name!!!!!")
        else:
            r = self.mir.mission_queue(guid.encode('utf-8'))
            print(r.text)

    def get_mir_status(self):
        r = self.mir.get_status()
        rjson = json.loads(r.text)
        d = {
            "mir_state": rjson.get("state_text").encode('utf-8'),
            "mir_position": {
                "x": rjson.get("position").get("x"),
                "y": rjson.get("position").get("y"),
                "yaw": rjson.get("position").get("yaw")
            }
        }
        return d
