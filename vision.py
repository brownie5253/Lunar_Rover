import io
import time
import picamera
import cv2
import numpy as np
import math
import dis

focal_length = 3.04
camera_height = 70
lander_height = 54.64
h = 313
w = 410
px_size = 0.00112*(3280/w);
px_angle_x = 62/w
px_angle_y = 48.8/h
def capture_image():
    #Create the in-memory stream
    stream = io.BytesIO()
    with picamera.PiCamera() as camera:
        camera.resolution = (w,h)
        camera.start_preview()
        camera.capture(stream, format='jpeg')
        #Construct a numpy array from the stream
    data = np.fromstring(stream.getvalue(), dtype=np.uint8)
    frame = cv2.imdecode(data, 1)
    frame = cv2.flip(frame, 0)
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_hsv = cv2.GaussianBlur(frame_hsv, (7, 7), 0)
    return frame_hsv

def detect_sample(frame_hsv):
  SAMPLE_MAX = (19, 248, 255)
  SAMPLE_MIN = (0, 150, 20) 
  sample_mask= cv2.inRange(frame_hsv, SAMPLE_MIN, SAMPLE_MAX)
  return sample_mask


def getReading():
  frame_hsv = capture_image()
  sample_mask = detect_sample(frame_hsv)
  cv2.imshow('', sample_mask)
  im2, cont_samples, hierarchy = cv2.findContours(sample_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  if cont_samples == []:
      return [None, None]

  for i in cont_samples:
    sample_distance = None
    x_angle = None
    if cv2.contourArea(i)>10:
      ((x, y), radius) = cv2.minEnclosingCircle(i)
      x = np.round(x).astype("int")
      y = np.round(y).astype("int")
      radius_int = np.round(radius).astype("int")
      percieved_diameter = radius * 2 * px_size
      real_diameter = 42.67
      direct_distance = (focal_length * real_diameter) / (percieved_diameter)
      #print(direct_distance)
      if direct_distance<camera_height:
        sample_distance = np.round(camera_height * math.cos(math.asin((direct_distance) / camera_height))).astype("int")
      else:
        sample_distance = np.round( direct_distance* math.cos(math.asin((camera_height) / direct_distance))).astype("int")
      #print(sample_distance)
      #print("hey")
      #distance_txt = "Distance:" + str(sample_distance) + " mm"
      #samples = cv2.circle(frame, (x, y), radius_int, (0, 0, 255), 1)
      x_angle = np.round(x * px_angle_x).astype("int")
      y_angle = np.round(y * px_angle_y).astype("int")
      #samples = cv2.putText(samples, "Sample " + distance_txt, (x - 25, y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
       #                     (0, 0, 255), 1)
      #samples = cv2.putText(samples, "x angle: " + str(x_angle) + ", y angle: " + str(y_angle), (x - 25, y - 25),
    #else:
     # sample_distance = None
      #x_angle = None
                      #        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
  #cv2.imshow("Image", sample_mask)
  #cv2.waitKey(0)
  
  cv2.destroyAllWindows
  return [sample_distance, x_angle]

  
