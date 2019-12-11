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

  return binary_img

def remove_shadows(img):
    rgb_planes = cv.split(img)

    result_planes = []
    result_norm_planes = []
    for plane in rgb_planes:
        dilated_img = cv.dilate(plane, np.ones((7,7), np.uint8))
        bg_img = cv.medianBlur(dilated_img, 21)
        diff_img = 255 - cv.absdiff(plane, bg_img)
        norm_img = cv.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8UC1)
        result_planes.append(diff_img)
        result_norm_planes.append(norm_img)

    result = cv.merge(result_planes)
    result_norm = cv.merge(result_norm_planes)

    return result, result_norm

def calc_runs(arr, val1, val2):
    ret1 = []
    ret2 = []
    i = 0
    while(i < len(arr)):
        mark = i
        while(i < len(arr) and arr[i] == val1):
            i+=1
        ret1.append(i - mark)
        mark = i
        while(i < len(arr) and arr[i] == val2):
            i+=1
        ret2.append(i - mark)
    return ret1, ret2

def dist_to_nearest(center, centers2, threshold=3):
  start = center
  dist = 0
  while((start + dist <= max(centers2) or start - dist >= 0) and dist < threshold):
      if (start - dist in centers2):
          return -1.0 * dist
      if (start + dist in centers2):
          return 1.0 * dist
      dist += 1
  return 0

def remove_isolated_pixels(image):
  connectivity = 8

  output = cv.connectedComponentsWithStats(image, connectivity, cv.CV_32S)

  num_stats = output[0]
  labels = output[1]
  stats = output[2]

  new_image = image.copy()

  for label in range(num_stats):
      if stats[label,cv.CC_STAT_AREA] < 20:
          new_image[labels == label] = 0

  return new_image

def remove_text(img):
  new_img = img.copy()
  kernel = cv.getStructuringElement(cv.MORPH_CROSS, (3,3))
  dilated = cv.dilate(new_img, kernel, iterations=5)
  _, contours, hierarchy = cv.findContours(dilated, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  for contour in contours:
      [x, y, w, h] = cv.boundingRect(contour)
      #if w < 35 and h < 35:
      #    continue
      if (w < width(img) // 2):
          cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), cv.FILLED)
  return img


  # calculate the lengths of successive runs of <val> in list <arr>
# def calc_runs(arr, val):
#     ret = []
#     i = 0
#     while(i < len(arr)):
#         while(i < len(arr) and arr[i] != val):
#             i+=1
#         mark = i
#         while(i < len(arr) and arr[i] == val):
#             i+=1
#         ret.append(i - mark)
#     return ret




# DEPRECATED
# def preprocess(img):
#   processed_img = img.copy()

#   # binary threshold
#   processed_img = binary_threshold(processed_img)

#   # rotate so staff is (approximately) horizontal
#   edges = cv.Canny(processed_img,50,150,apertureSize = 3)
#   show_wait_destroy("edges", edges)
#   minLineLength = 500
#   maxLineGap = 50
#   lines = cv.HoughLinesP(edges,1,np.pi/180,100,minLineLength=minLineLength,maxLineGap=maxLineGap)
#   if (lines is None):
#       raise RuntimeError("No staff lines found")
#   colored_img = cv.cvtColor(processed_img, cv.COLOR_GRAY2RGB)
#   skew_angles = []
#   for line in lines:
#     x1,y1,x2,y2 = line[0]
#     cv.line(colored_img,(x1,y1),(x2,y2),(0,255,0),2)
#     skew_angles.append(np.arctan2(y2 - y1, x2 - x1))

#   avg_skew = np.degrees(np.mean(skew_angles))
#   print("Average skew angle: ", np.mean(skew_angles))
#   show_wait_destroy("lines", colored_img)
#   M = cv.getRotationMatrix2D((width(processed_img) // 2, height(processed_img) // 2), avg_skew, 1)
#   processed_img = cv.warpAffine(processed_img, M, (width(processed_img), height(processed_img)))

#   # testing
#   edges = cv.Canny(processed_img,50,150,apertureSize = 3)
#   #show_wait_destroy("edges", edges)
#   #minLineLength = 20
#   #maxLineGap = 5
#   minLineLength = 300
#   maxLineGap = 30
#   lines = cv.HoughLinesP(edges,1,np.pi/180,100,minLineLength=minLineLength,maxLineGap=maxLineGap)
#   if (lines is None):
#       raise RuntimeError("No staff lines found")
#   #staff_segments = np.ones((height(processed_img),width(processed_img), 3), np.uint8) * 255
#   staff_segments = colored_img.copy()
#   for line in lines:
#     x1,y1,x2,y2 = line[0]
#     if (np.abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) < 5):
#       cv.line(staff_segments,(x1,y1),(x2,y2),(0,255,0),2)
#   show_wait_destroy("TEST", staff_segments)


#   return processed_img, cv.cvtColor(staff_segments, cv.COLOR_BGR2GRAY) / 255
