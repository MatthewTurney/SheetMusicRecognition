import cv2

# Apply binary thresholding to the image
# returned image will consist of values {0, 255}
def binary_threshold(img):

  # Otsu's thresholding - for removing background
  #_, binary_img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

  # use adaptive thresholding in this case to deal with potential shadows
  # 23 is block size, picked arbitrarily
  # 2 is constant subtracted from mean
  binary_img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 23, 2)
  return binary_img