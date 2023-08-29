import cv2
import numpy as np
import yaml

# Load the YAML file
with open("/home/asslam/.ros/camera_info/head_camera.yaml", "r") as file:
    data = yaml.load(file, Loader=yaml.FullLoader)

# Extract the camera matrix and distortion coefficients from the data
camera_matrix = np.array(data["camera_matrix"]["data"])
dist_coeffs = np.array(data["distortion_coefficients"]['data'])
print(camera_matrix)


# Load an image
img = cv2.imread("/home/asslam/Pictures/Webcam/2023-02-09-000919.jpg")
img = cv2.resize(img, (640, 480), interpolation = cv2.INTER_LINEAR)
h,  w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coeffs, (w,h), 1, (w,h))
# undistort
dst = cv2.undistort(img, camera_matrix,dist_coeffs, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png', dst)
# Show the original and undistorted images
cv2.imshow("Original Image", dst)
# cv2.imshow("Undistorted Image", undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()