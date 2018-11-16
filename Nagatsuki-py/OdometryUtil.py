import math

def truncate(value, number):
  output = math.floor(value * math.pow(10, number)) / math.pow(10, number)
  return output

class PinholeCamera:
  def __init__(self, width, height, fx, fy, cx, cy, k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
    self.width = width
    self.height = height
    self.fx = fx
    self.fy = fy
    self.cx = cx
    self.cy = cy
    self.distortion = (abs(k1) > 0.0000001)
    self.d = [k1, k2, p1, p2, k3]
