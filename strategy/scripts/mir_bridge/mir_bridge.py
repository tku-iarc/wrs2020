#!/usr/bin/python
import re
import json
import requests
import warnings
import math
import time
import base64
import os
from uuid import UUID

MAP_NAME = "HOME_AREA"

## TODO: Try to use function to reconstruct path for decorator's argument 'path'

class Request(object):
    def __init__(self, method, path):
        self.method = method.upper()
        self.path = path
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
                if payload.has_key("PATH"):
                    URL = "{}{}".format(URL, payload.get("PATH"))

                if payload.has_key("BODY"):
                    payload = payload.get("BODY")

                try:
                    res = requests.request(self.method,
                                           url=URL,
                                           headers=self.h,
                                           data=json.dumps(payload),
                                           timeout=3)
                except requests.RequestException as e:
                    print(e)
                    return
            else:
                try:
                    res = requests.request(self.method,
                                           url=URL,
                                           headers=self.h,
                                           timeout=3)
                except requests.RequestException as e:
                    print(e)
                    return
            return res
        return wrapper


class MIR(object):
    def __init__(self, host):
        if not "http" in host:
            warnings.warn("WARRING: Maybe the host name is error.")

        self.host = host

    def check_response_status_code(self, res):
        if str(res.status_code).startswith("20"):
            print("[Response OK]")
        else:
            raise Exception("Response ERROR")

    def is_valid_guid(self, guid_to_test, version=1):
        try:
            guid_obj = UUID(guid_to_test, version=version)
        except ValueError:
            return False

        return str(guid_obj) == guid_to_test

    @Request(method="get", path="/status")
    def get_status(self):
        pass

    @Request(method="put", path="/status")
    def set_status(self, set_state):
        STATE = {"Ready": 3, "Pause": 4}

        # Check whether input 'set_state' is correct
        if isinstance(set_state, (str, int)):
            if isinstance(set_state, str):
                s = STATE.get(set_state, 4)
        else:
            print("ERROR type of set_state")
            return

        body = {
            "state_id": s
        }
        return body

    @Request(method="get", path="/system/info")
    def get_system_info(self):
        pass

    @Request(method="get", path="/missions")
    def get_missions(self):
        pass

    def get_mission_guid(self, mission, auto_create=True):
        r = self.get_missions()
        rjson = json.loads(r.text)
        for l in rjson:
            if l.get("name") == mission:
                return l.get("guid")
        warnings.warn("No this mission")
        return None

    @Request(method="get", path="/missions")
    def get_mission_actions(self, mission):
        mission_guid = self.get_mission_guid(mission)
        if mission_guid is None:
            print("[WARNING] No this mission. Creating mission {}".format(mission))
            r = self.create_new_mission(mission)
            rjson = json.loads(r.text)
            mission_guid = rjson.get("guid")

        return {"PATH": "/" + mission_guid + "/actions"}

    @Request(method="get", path="/mission_queue")
    def get_mission_queue(self):
        pass

    @Request(method="post", path="/mission_queue")
    def post_mission_queue(self, mission):
        body = {
            "mission_id": mission
        }
        return body

    def add_mission_to_queue(self, mission):
        if not self.is_valid_guid(mission):
            mission_id = self.get_mission_guid(mission)
        r = self.post_mission_queue(mission_id)
        self.check_response_status_code(r)

    @property
    def mission_queue_is_empty(self):
        r = self.get_mission_queue()
        rjson = json.loads(r.text)
        for l in rjson:
            if l.get("state").upper() == "PENDING" or \
               l.get("state").upper() == "EXECUTING":
               return False
        return True

    @Request(method="delete", path="/mission_queue")
    def clear_mission_queue(self):
        pass

    @Request(method="get", path="/mission_groups")
    def get_groups(self):
        pass

    def get_group_guid(self, group):
        r = self.get_groups()
        rjson = json.loads(r.text)
        for l in rjson:
            if l.get("name") == group:
                return l.get("guid")
        warnings.warn("No this group")
        #print(r.text)
        return None

    @Request(method="get", path="/sessions")
    def get_sessions(self):
        pass

    def get_session_guid(self, session):
        r = self.get_sessions()
        rjson = json.loads(r.text)
        for l in rjson:
            if l.get("name") == session:
                return l.get("guid")
        warnings.warn("No this session")
        #print(r.text)
        return None

    @Request(method="post", path="/missions")
    def create_new_mission(self, name, group_id="TKU_IARC", session_id="TKU_IARC"):
        print("Creating new mission name: {}, group_id: {}, session_id: {}" \
                .format(name, group_id, session_id))
        if not self.is_valid_guid(group_id):
            group_id = self.get_group_guid(group_id)
        if not self.is_valid_guid(session_id):
            session_id = self.get_session_guid(session_id)

        body = {
            "group_id": group_id,
            "session_id": session_id,
            "name": name
        }
        return body

    @Request(method="delete", path="/missions")
    def delete_mission(self, mission):
        if not self.is_valid_guid(mission):
            mission_id = self.get_mission_guid(mission)
            if mission_id is None:
                warnings.warn("No this mission to DELETE")
                return
        return {"PATH": "/" + mission_id}

    @Request(method="post", path="/missions")
    def add_action_to_mission(self, mission, action_type, parameters, priority, scope_reference=None):
        mission_guid = self.get_mission_guid(mission)
        if mission_guid is None:
            print("[WARNING] No this mission. Creating mission {}".format(mission))
            r = self.create_new_mission(mission)
            rjson = json.loads(r.text)
            mission_guid = rjson.get("guid")

        path = "/" + mission_guid + "/actions"
        body = {
            "action_type": action_type,
            "mission_id": mission_guid,
            "parameters": parameters,
            "priority": priority,
            "scope_reference": scope_reference
        }
        return {"PATH": path, "BODY": body}

    def add_try_catch_action(self, priority):
        param = [
            { "id": "try",  "value": ""}, 
            { "id": "catch",  "value": ""}
        ]
        self.add_action_to_mission("TKU_TMP", "try_catch", param, priority)

    def get_scope_reference_guid(self, mission, id):
        r = self.get_mission_actions(mission)
        rjson = json.loads(r.text)
        for l in rjson:
            for i in l.get("parameters"):
                if i.get("id") == id:
                    return i.get("guid")

        warnings.warn("No scope_reference")
        return None

    def add_relative_move_action(self, dx=0.0, dy=0.0, dyaw=0.0, \
                                 max_speed_v=0.5, max_speed_w=0.5, \
                                 collision_detection=True, priority=1, \
                                 use_try_catch=True):
        scope_reference = None
        param = [
            { "id": "x", "value": dx},
            { "id": "y", "value": dy},
            { "id": "orientation", "value": dyaw},
            { "id": "max_linear_speed", "value": max_speed_v},
            { "id": "max_angular_speed", "value": max_speed_w},
            { "id": "collision_detection", "value": collision_detection}
        ]
        if use_try_catch:
            self.add_try_catch_action(1)
            sound_param = [
                { "id": "sound",  "value": "mirconst-guid-0000-0001-sounds000000"}, 
                { "id": "volume",  "value": 80.0}, 
                { "id": "mode",  "value": "custom"}, 
                { "id": "duration",  "value": "00:00:03.000000"}
            ]
            scope_ref_try = self.get_scope_reference_guid("TKU_TMP", "try")
            scope_ref_catch = self.get_scope_reference_guid("TKU_TMP", "catch")
            self.add_action_to_mission("TKU_TMP", "sound", sound_param, 3, scope_ref_catch)
            self.add_action_to_mission("TKU_TMP", "relative_move", param, priority, scope_ref_catch)
            self.add_action_to_mission("TKU_TMP", "relative_move", param, priority, scope_ref_try)
        else:
            self.add_action_to_mission("TKU_TMP", "relative_move", param, priority)


    def relative_move(self, dx=0.0, dy=0.0, dyaw=0.0, \
                      max_speed_v=0.5, max_speed_w=0.5, collision_detection=True):
        self.clear_mission_queue()
        self.delete_mission("TKU_TMP")
        self.add_relative_move_action(dx, dy, dyaw, max_speed_v, max_speed_w, collision_detection, 1)
        self.add_mission_to_queue("TKU_TMP")

    @Request(method="get", path="/positions")
    def get_positions(self):
        pass

    @Request(method="get", path="/positions")
    def get_position_by_id(self, guid):
        return {"PATH": "/" + guid}

    def get_position_guid(self, position_name):
        r = self.get_positions()
        self.check_response_status_code(r)
        rjson = json.loads(r.text)
        for l in rjson:
            if l.get("name") == position_name:
                return l.get("guid")
        print("No this position")
        return None

    @Request(method="post", path="/positions")
    def add_position(self, name, x, y, yaw, position_type=0):
        map_guid = self.get_map_guid(MAP_NAME)
        body = {
            "map_id": map_guid,
            "name": name,
            "orientation": yaw,
            "pos_x": x,
            "pos_y": y,
            "type_id": position_type
        }
        return body

    @property
    def status(self):
        r = self.get_status()
        self.check_response_status_code(r)
        rjson = json.loads(r.text)
        d = {
            "mir_state": rjson.get("state_text").encode('utf-8'),
            "mir_position": {
                "x": rjson.get("position").get("x"),
                "y": rjson.get("position").get("y"),
                "yaw": rjson.get("position").get("orientation")
            }
        }
        return d

    ## Maybe this function is useless
    def arrived_position(self, position_name):
        id = self.get_position_guid(position_name)
        r = self.get_position_by_id(id)
        self.check_response_status_code(r)
        rjson = json.loads(r.text)

        if rjson.get("name") == position_name:
            rs = self.get_status()
            rsjson = json.loads(rs.text)
            dx = rjson.get("pos_x") - rsjson.get("position").get("x")
            dy = rjson.get("pos_y") - rsjson.get("position").get("y")
            dyaw = rjson.get("orientation") - rsjson.get("position").get("orientation")

            if math.hypot(dx, dy) < 0.1 and abs(dyaw) < 10:
                print("Distanse is short enough. {}, {}, {}".format(dx, dy, dyaw))
                return True
            else:
                return False

    @Request(method="get", path="/maps")
    def get_maps(self):
        pass

    @Request(method="get", path="/maps")
    def get_map(self, map_name):
        if not self.is_valid_guid(map_name):
            map_guid = self.get_map_guid(map_name)
            if map_guid is None:
                warnings.warn("No this map: {}".format(map_name))
                return
        return {"PATH": "/" + map_guid}

    def get_map_guid(self, map_name):
        r = self.get_maps()
        rjson = json.loads(r.text)
        for l in rjson:
            if l.get("name") == map_name:
                return l.get("guid")
        print("No this position")
        return None

    def save_map(self, map_name=MAP_NAME, saved_name=None, \
                 saved_path=os.path.dirname(os.path.abspath(__file__))+"/maps/"):
        if saved_name is None:
            t = time.localtime()
            timestamp = time.strftime('%b-%d-%Y_%H%M', t)
            saved_name = (map_name + "-" + timestamp + ".png")
        r = self.get_map(map_name)
        rjson = json.loads(r.text)
        bMap = rjson.get("map")
        print(bMap)

        if not os.path.exists(saved_path):
            os.mkdir(saved_path)
            print("Directory " , saved_path,  " Created ")

        with open(saved_path + saved_name, "wb") as fh:
            fh.write(base64.b64decode(bMap))
        print("[INFO] Saved {} map in {}".format(map_name, saved_path))
