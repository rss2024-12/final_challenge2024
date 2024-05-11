import cv2
import numpy as np
import sys

def detect_and_draw_line(image_path):
    # Load the PNG image
    print("Image path:", image_path)
    cv_img = cv2.imread(image_path)
    
    # Convert BGR image to HSV
    hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for the line color in HSV
    lower_bound = np.array([0, 150, 200])  # Lower bound for white in HSV [0,55,106] to [11,255,255]
    upper_bound = np.array([10, 255, 255])  # Upper bound for white in HSV
    
    # Ignore the top half of the image
    height,width = cv_img.shape[:2]
    
    #self.get_logger().info(f'Height:{height}')
    top_mask = np.zeros_like(hsv_img[:, :, 0])
    top_mask[height//2:, :] = 255
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
    mask = cv2.bitwise_and(mask, top_mask)
    mask_resized = cv2.resize(mask, (800, 600))
    cv2.imshow("Mask", mask_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
##
    # Apply the Hough Line Transform to detect lines in the masked image


    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize bounding box
    box = ((0, 0), (0, 0))
    
    # Iterate through contours and find the bounding box of the largest contour
    if len(contours) > 0:
        
        largest_contour = max(contours, key=cv2.contourArea)
        print(largest_contour)
        x, y, w, h = cv2.boundingRect(largest_contour)
        box = ((x, y), (x + w, y + h))
    
        mid_x=(box[0][0]+box[1][0])//2 #takes the middle 
        mid_y=(box[1][0]+box[1][1])//2#grabs the point on the bottom of the bounding box
      
        cv_img = cv2.circle(cv_img, (mid_x, mid_y), 10, (255, 0, 0), -1)
        
      
    output_image_path = "output_image.jpeg"
    cv2.imwrite(output_image_path, cv_img)
 

    img_resized = cv2.resize(cv_img, (800, 600))
    cv2.imshow("Image with dot", img_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
# Path to the PNG image
if len(sys.argv) != 2:
    print("Usage: python3 test.py /path/to/image")
else:
    # Using raw string literal for Windows path
    image_path = r"C:\Users\Luis A Garcia\racecar_docker\home\racecar_ws\src\final_challenge2024\luigi_mansion\luigi_mansion\stop_detector\stop_off.jpg"
    detect_and_draw_line(image_path)