<<<<<<< HEAD
import cv2
=======
import cv2 as cv
>>>>>>> cbdb48692040ec300312c772377572e1c4504edd

# Apply binary thresholding to the image
# returned image will consist of values {0, 255}
def binary_threshold(img):

  # Otsu's thresholding - for removing background
  #_, binary_img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

  # use adaptive thresholding in this case to deal with potential shadows
  # 23 is block size, picked arbitrarily
  # 2 is constant subtracted from mean
<<<<<<< HEAD
  binary_img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 23, 2)
=======
  binary_img = img.copy()
  cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 23, 2, dst=binary_img)
>>>>>>> cbdb48692040ec300312c772377572e1c4504edd
  return binary_img