import pyrealsense2 as rs

class getRealsense:
  def __init__(self, width, height):
    self.deviceDetect = False
    self.EnableColor = False
    self.EnableInfrared = False
    self.EnableDepth = False
    self.printPara = False
    self.width = width
    self.height = height
  
  def detectDevice(self):
    ctx = rs.context()
    devices = ctx.query_devices()

    for dev in devices:
      self.productName = str(dev.get_info(rs.camera_info.product_id))
      
      # DS 435 config
      if self.productName in "0B07":
        self.EnableColor = True
        self.EnableInfrared = True
        self.EnableDepth = True
        self.deviceDetect = True
        break
      
      # DS 415 config
      elif self.productName in "0AD3":
        self.EnableColor = True
        self.EnableInfrared = True
        self.EnableDepth = True
        self.deviceDetect = True
        break
          
      # DS 410 config
      elif self.productName in "0AD2":
        self.EnableColor = False
        self.EnableInfrared = True
        self.EnableDepth = True
        self.deviceDetect = True
        break

    if self.deviceDetect is not True:
        raise Exception("No supported device was found")

  def getParameter(self):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    if self.EnableColor is True:
        config.enable_stream(rs.stream.color, int(self.width), int(self.height), rs.format.bgr8, 30)

    if self.EnableInfrared is True:
        config.enable_stream(rs.stream.infrared, 1, int(self.width), int(self.height), rs.format.y8, 30)
        config.enable_stream(rs.stream.infrared, 2, int(self.width), int(self.height), rs.format.y8, 30)
        
    if self.EnableDepth is True:
        config.enable_stream(rs.stream.depth, int(self.width), int(self.height), rs.format.z16, 30)
        
    # Start streaming
    cfg = pipeline.start(config)

    # Get Video Stream Intrinsics
    if self.printPara is True:
      print("=======================================================")
      print("Get Video Stream Intrinsics")
      print("=======================================================")
      
    if self.EnableColor is True:
      colorStream = cfg.get_stream(rs.stream.color)
      colorProfile = colorStream.as_video_stream_profile()
      self.colorIntrin = colorProfile.get_intrinsics()
      if self.printPara is True:
        print("Color Intrinsics:")
        print(self.colorIntrin)
        print("")

    if self.EnableInfrared is True:
      infraredStream = cfg.get_stream(rs.stream.infrared, 1)
      infraredProfile = infraredStream.as_video_stream_profile()
      self.infraredIntrin = infraredProfile.get_intrinsics()
      if self.printPara is True:
        print("Infrared Intrinsics:")
        print(self.infraredIntrin)
        print("")

    if self.EnableDepth is True:
      depthStream = cfg.get_stream(rs.stream.depth)
      depthProfile = depthStream.as_video_stream_profile()
      self.depthIntrin = depthProfile.get_intrinsics()
      if self.printPara is True:
        print("Depth Intrinsics:")
        print(self.depthIntrin)
        print("")

      # Get and Apply Depth-to-Stream Extrinsics
        print("\n=======================================================")
        print("Get and Apply Depth-to-Stream Extrinsics")
        print("=======================================================")
        print("Color Extrinsics:")
      
      colorExtrin = depthStream.get_extrinsics_to(colorStream)
      if self.printPara is True:
        print(colorExtrin)
        print("")
        print("Infrared Extrinsics:")
      
      infraredExtrin = depthStream.get_extrinsics_to(infraredStream)
      if self.printPara is True:
        print(infraredExtrin)

    if self.EnableInfrared is True:
      # Get Disparity Baseline
      if self.printPara is True:
        print("\n=======================================================")
        print("Get Disparity Baseline")
        print("=======================================================")
      
      infraredStream2 = cfg.get_stream(rs.stream.infrared, 2)
      self.baseline = infraredStream2.get_extrinsics_to(infraredStream)
      if self.printPara is True:
        print(self.baseline)
    
    pipeline.stop()
  
  def configYAML(self, type):
    if type in ["RGB"]:
      if self.productName in "0B07": # DS 435 config
        self.printYAML(self.colorIntrin,self. baseline)
      elif self.productName in "0AD3": # DS 415 config
        self.printYAML(self.colorIntrin, self.baseline)
      elif self.productName in "0AD2": # DS 410 config
        self.printYAML(self.infraredIntrin, self.baseline)
    elif type in ["stereo"]:
      if self.productName in "0B07": # DS 435 config
        self.printYAML(self.infraredIntrin, self.baseline)
      elif self.productName in "0AD3": # DS 415 config
        self.printYAML(self.infraredIntrin, self.baseline)
      elif self.productName in "0AD2": # DS 410 config
        self.printYAML(self.infraredIntrin, self.baseline)
    elif type in ["RGBD"]:
      if self.productName in "0B07": # DS 435 config
        self.printYAML(self.colorIntrin,self. baseline)
      elif self.productName in "0AD3": # DS 415 config
        self.printYAML(self.colorIntrin, self.baseline)
  
  
  def printYAML(self, intrin, baseline):
    print("\n=======================================================")
    print("YAML")
    print("=======================================================")
    print("Camera.fx: " + str(intrin.fx))
    print("Camera.fy: " + str(intrin.fy))
    print("Camera.cx: " + str(intrin.ppx))
    print("Camera.cy: " + str(intrin.ppy))
    print("")
    print("Camera.k1: " + str(intrin.coeffs[0]))
    print("Camera.k2: " + str(intrin.coeffs[1]))
    print("Camera.p1: " + str(intrin.coeffs[2]))
    print("Camera.p2: " + str(intrin.coeffs[3]))
    print("Camera.k3: " + str(intrin.coeffs[4]))
    print("")
    print("Camera.width: " + str(intrin.width))
    print("Camera.height: " + str(intrin.height))
    print("")
    
    bf = intrin.fx * baseline.translation[0]
    print("Camera.bf: " + str(bf))
    
  
if __name__ == '__main__':
  from argparse import ArgumentParser
  
  parser = ArgumentParser()
  parser.add_argument("-width", "--width", dest="width", default=640)
  parser.add_argument("-height", "--height", dest="height", default=480)
  parser.add_argument("-type", "--type", dest="type", default="RGB")
  args = parser.parse_args()
  
  getRS = getRealsense(args.width, args.height)
  getRS.detectDevice()
  getRS.getParameter()
  getRS.configYAML(args.type)