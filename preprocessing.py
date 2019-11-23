import cv2 as cv

# Apply binary thresholding to the image
# returned image will consist of values {0, 255}
def binary_threshold(img):

  # Otsu's thresholding - for removing background
  _, binary_img = cv.threshold(img, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)

  # use adaptive thresholding in this case to deal with potential shadows
  # 23 is block size, picked arbitrarily
  # 2 is constant subtracted from mean
  #binary_img = img.copy()
  #cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 23, 2, dst=binary_img)
  return binary_img