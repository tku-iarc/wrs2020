#!/usr/bin/python
import json
import requests

class MIR():
  def __init__(self, host):
    self._host = host

  def Request(self, method, path):
    URL = ''.join([self._host, path])

    h = {
      "Content-Type": "application/json;charset=UTF-8",
      "Accept-Language": "en-US",
      "Authorization": "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
    }

    res = getattr(requests, method)(url = URL, headers = h)

    if res.status_code.startswith('20'):
      print(res.text)
    elif res.status_code.startswith('40'):
      print(res.text)
    else:
      print("Unknown ERROR")