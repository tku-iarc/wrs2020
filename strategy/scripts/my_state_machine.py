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
        r = self.mir.status("Pause")
        print(r)

    def on_toMove(self, position):
        self.mir.status("Ready")
        print("to Move {}".format(position))

        if position is "ROOMA":
            r = self.mir.mission_queue("0bec3a34-4f56-11ea-82bd-f44d30609d1f")
        elif position is "HOME":
            r = self.mir.mission_queue("6c94d08a-4f59-11ea-82bd-f44d30609d1f")
        else:
            print("Unknown position")

        print(r)

    def get_mir_status(self):
        r = self.mir.get_status()
        rjson = json.loads(r.text)
        d = {
            "mir_state": rjson.get("state_text"),
            "mir_position": {
                "x": rjson.get("position").get("x"),
                "y": rjson.get("position").get("y"),
                "yaw": rjson.get("position").get("yaw"),
            }
        }
        return d
