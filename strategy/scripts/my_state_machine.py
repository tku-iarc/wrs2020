from statemachine import StateMachine, State
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from strategy.cfg import RobotConfig
from my_ros_bridge.my_ros_bridge import Robot

class MyStateMachine(Robot, StateMachine):

  def __init__(self, sim = False):
    super(MyStateMachine, self).__init__(sim)
    StateMachine.__init__(self)
    dsrv = DynamicReconfigureServer(RobotConfig, self.Callback)

  def Callback(self, config, level):
    self.start = config['start']

    return config

  idle   = State('Idle', initial = True)
  move   = State('Move')

  toIdle   = move.to(idle) | idle.to.itself()
  toMove   = idle.to(move)| move.to.itself()

  def on_toIdle(self):
    pass

  def on_toMove(self):
    pass