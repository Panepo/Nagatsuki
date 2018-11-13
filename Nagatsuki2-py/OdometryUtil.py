import math

def truncate(value, number):
  output = math.floor(value * math.pow(10, number)) / math.pow(10, number)
  return output
