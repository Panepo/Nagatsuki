import numpy as np
import cv2

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

lk_params = dict(winSize = (21,21), maxLevel = 3, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(img_1, img_2, p1):
  p2, st, err = cv2.calcOpticalFlowPyrLK(img_1, img_2, p1, None, **lk_params)
  st = st.reshape(st.shape[0])
  p1 = p1[st==1]
  p2 = p2[st==1]

  return p1,p2

def featureDetection():
  thresh = dict(threshold=25, nonmaxSuppression=True)
  fast = cv2.FastFeatureDetector_create(**thresh)
  return fast

class VisualOdometry:
  def __init__(self, cam, dataset):
    self.frame_stage = 0
    self.cam = cam
    self.new_frame = None
    self.last_frame = None
    self.cur_R = None
    self.cur_t = [0, 0, 0]
    self.px_ref = None
    self.px_cur = None
    self.focal = cam.fx
    self.pp = (cam.cx, cam.cy)
    self.dataset = False
    self.trueX, self.trueY, self.trueZ = 0, 0, 0
    self.detector = featureDetection()
    self.prob = 0.999
    self.thre = 1.0
    self.scale = 1.0

  def readAnnotations(self, annotations):
    with open(annotations) as f:
      self.annotations = np.genfromtxt(f, delimiter=' ',dtype=None)

  def setParameter(self, prob, thre, scale):
    self.prob = prob
    self.thre = thre
    self.scale = scale

  def getAbsoluteScale(self, frame_id):  #specialized for KITTI odometry dataset
    ss = self.annotations[frame_id-1].strip().split()
    x_prev = float(ss[3])
    y_prev = float(ss[7])
    z_prev = float(ss[11])
    ss = self.annotations[frame_id].strip().split()
    x = float(ss[3])
    y = float(ss[7])
    z = float(ss[11])
    self.trueX, self.trueY, self.trueZ = x, y, z
    return np.sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev))

  def processFirstFrame(self):
    self.px_ref = self.detector.detect(self.new_frame)
    self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
    self.frame_stage = STAGE_SECOND_FRAME

  def processSecondFrame(self):
    self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
    E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=self.prob, threshold=self.thre)
    _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
    self.frame_stage = STAGE_DEFAULT_FRAME
    self.px_ref = self.px_cur

  def processFrame(self, frame_id):
    if (len(self.px_ref) < kMinNumFeature):
      self.px_ref = self.detector.detect(self.last_frame)
      self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)

    self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
    E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=self.prob, threshold=self.thre)
    _, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)

    if (self.dataset):
      absolute_scale = self.getAbsoluteScale(frame_id)
      if(absolute_scale > 0.1):
        self.cur_t = self.cur_t + absolute_scale*self.cur_R.dot(t)
        self.cur_R = R.dot(self.cur_R)
    else:
      self.cur_t = self.cur_t + self.scale * self.cur_R.dot(t)
      self.cur_R = R.dot(self.cur_R)

    self.px_ref = self.px_cur

  def checkImage(self, img):
    if len(img) == 3:
      return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
      return img

  def update(self, img, frame_id):
    self.new_frame = self.checkImage(img)
    if(self.frame_stage == STAGE_DEFAULT_FRAME):
      self.processFrame(frame_id)
    elif(self.frame_stage == STAGE_SECOND_FRAME):
      self.processSecondFrame()
    elif(self.frame_stage == STAGE_FIRST_FRAME):
      self.processFirstFrame()
    self.last_frame = self.new_frame
