import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template, blackout_regions=False):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
    # Convert BGR image to HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for the line color in HSV
    lower_bound = np.array([0, 0, 200])  # Lower bound for orange in HSV [0,55,106] to [11,255,255]
    upper_bound = np.array([179, 50, 255])  # Upper bound for orange in HSV
    
    # Create a mask using the defined bounds
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

    if blackout_regions:
        height, width = mask.shape
        fifth_height = int(height / 5)
        two_fifths_height = int(2 * height / 5)
        mask[:fifth_height, :] = 0  # Black out top 1/5
        mask[3*fifth_height:, :] = 0  # Black out bottom 2/5
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize bounding box
    bounding_box = ((0, 0), (0, 0))
    
    # Iterate through contours and find the bounding box of the largest contour
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        bounding_box = ((x, y), (x + w, y + h))
       
    # Return bounding box
    return bounding_box

def draw_bounding_box_file(file):
    """
    Draw bounding box on the original image.
    Input:
        img: np.3darray; the original input image.
        bounding_box: ((x1, y1), (x2, y2)); the bounding box to be drawn.
                      (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox.
    Return:
        img_with_bbox: np.3darray; the original image with the bounding box drawn in red.
    """
    img = cv2.imread(file)
    bounding_box = cd_color_segmentation(img, 0)
    
    # Make a copy of the original image
    img_with_bbox = img.copy()
    
    # Extract coordinates of the bounding box
    (x1, y1), (x2, y2) = bounding_box
    
    # Draw the bounding box on the image
    cv2.rectangle(img_with_bbox, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw in red
    
    cv2.imshow("Image with Bounding Box", img_with_bbox)
    cv2.waitKey(0)

def draw_bounding_box(img):
    """
    Draw bounding box on the original image.
    Input:
        img: np.3darray; the original input image.
        bounding_box: ((x1, y1), (x2, y2)); the bounding box to be drawn.
                      (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox.
    Return:
        img_with_bbox: np.3darray; the original image with the bounding box drawn in red.
    """
    bounding_box = cd_color_segmentation(img, 0)
    
    # Make a copy of the original image
    img_with_bbox = img.copy()
    
    # Extract coordinates of the bounding box
    (x1, y1), (x2, y2) = bounding_box
    
    # Draw the bounding box on the image
    cv2.rectangle(img_with_bbox, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw in red

    return img_with_bbox

def draw_mask(img):
    """
    Draw bounding box on the original image.
    Input:
        img: np.3darray; the original input image.
        bounding_box: ((x1, y1), (x2, y2)); the bounding box to be drawn.
                      (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox.
    Return:
        img_with_bbox: np.3darray; the original image with the bounding box drawn in red.
    """
    # Convert BGR image to HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for the cone color in HSV
    lower_bound = np.array([5, 50, 50])  # Lower bound for orange in HSV
    upper_bound = np.array([15, 255, 255])  # Upper bound for orange in HSV
    
    # Create a mask using the defined bounds
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

    return mask
    
# draw_bounding_box('./test_images_cone/test12.jpg')
