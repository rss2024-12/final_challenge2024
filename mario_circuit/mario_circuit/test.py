import cv2
import numpy as np

def detect_and_draw_line(image_path):
    # Load the PNG image
    img = cv2.imread(image_path)
    
    # Convert BGR image to HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for the line color in HSV
    lower_bound = np.array([0, 0, 200])  
    upper_bound = np.array([179, 50, 255])  
   
    # Threshold the HSV image to get a binary mask
 # Create a mask for the top half of the image
    height, width = img.shape[:2]
   
    ###ADD to ROBOT
    trapezoid_mask = np.zeros_like(hsv_img[:, :, 0])  # Rename mask to trapezoid_mask
    trap_top_width = int(0.5 * width)  # Width of the top of the trapezoid
    trap_bottom_width = width  # Width of the bottom of the trapezoid
   

    # Define the vertices of the trapezoid
    vertices = np.array([[(width - trap_bottom_width) // 2, 3 * height // 4],  # Bottom left
                        [width - (width - trap_bottom_width) // 2, 3 * height // 4],  # Bottom right
                        [width - (width - trap_top_width) // 2, 2*height // 4],  # Top right
                        [(width - trap_top_width) // 2, 2*height // 4]], dtype=np.int32)  # Top left

    # Fill the trapezoid with white (255)
    cv2.fillPoly(trapezoid_mask, [vertices], 255)

    # Threshold the HSV image to get a binary mask
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
    mask = cv2.bitwise_and(mask, trapezoid_mask) 
    ###END ADD to ROBOT
    # Apply the top mask
    mask = cv2.bitwise_and(mask, trapezoid_mask)
    kernel = np.ones((5,5),np.uint8)
    mask_dilated = cv2.dilate(mask, kernel, iterations=1)

    mask_resized = cv2.resize(mask_dilated, (800, 600))
    cv2.imshow("Mask", mask_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Apply the Hough Line Transform to detect lines in the masked image
    edges = cv2.Canny(mask_dilated, 50, 150)
    lines = cv2.HoughLines(edges, rho=1, theta=np.pi/180, threshold=200)

    max_slope_line = None
    max_slope = -np.inf

    # Iterate through the detected lines
   
    if lines is not None:
        
        for line in lines:
            rho, theta = line[0]
            

            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
                # Calculate the endpoints of the line
            x1 = int(x0 + 10000 * (-b))  # Extend the line by a large factor for visualization
            y1 = int(y0 + 10000 * (a))
            x2 = int(x0 - 10000 * (-b))
            y2 = int(y0 - 10000 * (a))
            #img = cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 5)  # Draw the line in green

            
            m = theta
            if m > max_slope:
                max_slope = m
                max_slope_line = line

    #print(max_slope_line)
    if max_slope_line is not None:
        rho, theta = max_slope_line[0]
        
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
            # Calculate the endpoints of the line
        x1 = int(x0 + 10000 * (-b))  # Extend the line by a large factor for visualization
        y1 = int(y0 + 10000 * (a))
        x2 = int(x0 - 10000 * (-b))
        y2 = int(y0 - 10000 * (a))

        #Calculate the point we want to return
        
        midline_y = height // 2

       # Cartestian Convert
        y_int = rho/np.sin(theta)
        m = -np.cos(theta)/np.sin(theta)

        mid_x = int((midline_y-y_int)/m)
        # print(x1,y1)
        # print(x2,y2)
        print(mid_x,midline_y)
        # print(img.shape)
        # Draw the line on the image
        img = cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 5)  # Draw the line in green
    
        # Draw the midpoint on the image
        img = cv2.circle(img, (mid_x, midline_y), 10, (255, 0, 0), -1)  # Draw a blue circle at the midpoint
        # Save the modified image
        output_image_path = "output_image.jpeg"
        cv2.imwrite(output_image_path, img)
    else:
        print("No line detected in the image.")

    img_resized = cv2.resize(img, (800, 600))
    cv2.imshow("Image with Drawn Line", img_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
# Path to the PNG image
image_path = "track2.jpeg"

# Call the function to detect and draw line
detect_and_draw_line(image_path)
