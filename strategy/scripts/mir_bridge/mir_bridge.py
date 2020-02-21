#!/usr/bin/python
import re
import json
import requests

class Request(object):
  def __init__(self, method, path):
    self.method = method.upper()
    self.path   = path
    self.h = {
      "Content-Type": "application/json;charset=UTF-8",
      "Accept-Language": "en-US",
      "Authorization": "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
    }

  def __call__(self, f):
    def wrapper(obj, *args, **kwargs):
      payload = f(obj, *args, **kwargs)
      URL = "{}{}".format(obj.host, self.path)
      if payload is not None:
        try:
          res = requests.request(self.method,                \
                                 url = URL,                  \
                                 headers = self.h,           \
                                 data = json.dumps(payload), \
                                 timeout = 3)
        except requests.RequestException as e:
          print(e)
          return
      else:
        try:
          res = requests.request(self.method,      \
                                 url = URL,        \
                                 headers = self.h, \
                                 timeout = 3)
        except requests.RequestException as e:
          print(e)
          return
      return res
    return wrapper

class MIR(object):
  def __init__(self, host):
    if not "http" in host:
      print("WARRING: Maybe the host name is error.")
    self.host = host

  @Request(method = "get", path = "/status")
  def GetStatus(self):
    pass

  @Request(method = "put", path = "/status")
  def Status(self, state):
    STATE = {"Ready": 3, "Pause": 4}
    if isinstance(state, (str, int)):
      if isinstance(state, str):
        s = STATE.get(state, 4)
    else:
      print("ERROR type of state")
      return
    body = {
      "state_id": s
    }
    return body

  @Request(method = "get", path = "/system/info")
  def SystemInfo(self):
    pass

  @Request(method = "get", path = "/positions")
  def Positions(self):
    pass

  @Request(method = "get", path = "/mission_queue")
  def MissionQueue(self):
    pass

  @Request(method = "post", path = "/mission_queue")
  def MissionQueue(self, mission):
    ## To ROOMA "0bec3a34-4f56-11ea-82bd-f44d30609d1f"
    ## To Home  "6c94d08a-4f59-11ea-82bd-f44d30609d1f"
    body = {
      "mission_id": mission
    }
    return body

    ## TODO: Clear Mission queue, Get position(sha) from name
    ## TODO: Positions
    ## TODO: Understand action to achieve related move?