import cv2 as cv
import numpy as np
from extensions import show_wait_destroy
from extensions import width, height

# Apply binary thresholding to the image
# returned image will consist of values {0, 255}
def binary_threshold(img):

  binary_img = img.copy()
  #binary_img = cv.bitwise_not(binary_img)

  # Otsu's thresholding - for removing background
  _, binary_img = cv.threshold(binary_img, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)

  # use adaptive thresholding in this case to deal with potential shadows
  # 23 is block size, picked arbitrarily
  # 2 is constant subtracted from mean
  #binary_img = img.copy()
  #cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 23, 2, dst=binary_img)
  return binary_img

def preprocess(img):
  processed_img = img.copy()

  # binary threshold
  processed_img = binary_threshold(processed_img)

  # rotate so staff is (approximately) horizontal
  edges = cv.Canny(processed_img,50,150,apertureSize = 3)
  show_wait_destroy("edges", edges)
  minLineLength = 500
  maxLineGap = 50
  lines = cv.HoughLinesP(edges,1,np.pi/180,100,minLineLength=minLineLength,maxLineGap=maxLineGap)
  if (lines is None):
      raise RuntimeError("No staff lines found")
  colored_img = cv.cvtColor(processed_img, cv.COLOR_GRAY2RGB)
  skew_angles = []
  for line in lines:
    x1,y1,x2,y2 = line[0]
    cv.line(colored_img,(x1,y1),(x2,y2),(0,255,0),2)
    skew_angles.append(np.arctan2(y2 - y1, x2 - x1))

  avg_skew = np.degrees(np.mean(skew_angles))
  print("Average skew angle: ", np.mean(skew_angles))
  show_wait_destroy("lines", colored_img)
  M = cv.getRotationMatrix2D((width(processed_img) // 2, height(processed_img) // 2), avg_skew, 1)
  processed_img = cv.warpAffine(processed_img, M, (width(processed_img), height(processed_img)))

  # testing
  edges = cv.Canny(processed_img,50,150,apertureSize = 3)
  #show_wait_destroy("edges", edges)
  #minLineLength = 20
  #maxLineGap = 5
  minLineLength = 300
  maxLineGap = 30
  lines = cv.HoughLinesP(edges,1,np.pi/180,100,minLineLength=minLineLength,maxLineGap=maxLineGap)
  if (lines is None):
      raise RuntimeError("No staff lines found")
  #staff_segments = np.ones((height(processed_img),width(processed_img), 3), np.uint8) * 255
  staff_segments = colored_img.copy()
  for line in lines:
    x1,y1,x2,y2 = line[0]
    if (np.abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) < 5):
      cv.line(staff_segments,(x1,y1),(x2,y2),(0,255,0),2)
  show_wait_destroy("TEST", staff_segments)


  return processed_img, cv.cvtColor(staff_segments, cv.COLOR_BGR2GRAY) / 255
