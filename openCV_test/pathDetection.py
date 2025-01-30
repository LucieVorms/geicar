import cv2
import numpy as np

def detect_and_draw_path(image_path):
    # Retrieve the image
    image = cv2.imread(image_path)
    
    # Pre-process the image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
    edges = cv2.Canny(blurred_image, threshold1=50, threshold2=200)

    # Find the contours 
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Divide the image in 4 parts: half in width, 2/3 and 1/3 in height
    height, width = edges.shape
    mid_x = width // 2
    half_y = 3 * height // 4
    exclude_band_y = int(height * 0.87)  # Exclude bottom 13% of the image

    # Categorize contours by part of the image, excluding the bottom band
    left_contours = [
        pt for contour in contours for pt in contour 
        if pt[0][0] < mid_x and half_y <= pt[0][1] < exclude_band_y
    ]
    right_contours = [
        pt for contour in contours for pt in contour 
        if pt[0][0] >= mid_x and half_y <= pt[0][1] < exclude_band_y
    ]

    # Initialize points to trace the line to delimit the right and left side of the path
    A1, A2, B1, B2 = None, None, None, None

    # Find the points to trace the line to delimit the right side of the path 
    if right_contours:
        A1 = min(right_contours, key=lambda pt: (pt[0][1], pt[0][0]))[0]  # Top + right
        B1 = max(right_contours, key=lambda pt: (pt[0][1], -pt[0][0]))[0]  # Bottom + right

    # Find the points to trace the line to delimit the left side of the path 
    if left_contours:
        A2 = min(left_contours, key=lambda pt: (pt[0][1], -pt[0][0]))[0]  # Top + left
        B2 = max(left_contours, key=lambda pt: pt[0][1])[0]  # Bottom + left

    # Draw the line and mark the point
    result_image = image.copy()
    if A1 is not None and B1 is not None:
        cv2.line(result_image, tuple(A1), tuple(B1), (255, 0, 0), 3)  # Blue line (right)
        cv2.circle(result_image, tuple(A1), 10, (255, 255, 0), -1)  # A1
        cv2.circle(result_image, tuple(B1), 10, (255, 255, 0), -1)  # B1
    if A2 is not None and B2 is not None:
        cv2.line(result_image, tuple(A2), tuple(B2), (0, 0, 255), 3)  # Red line (left)
        cv2.circle(result_image, tuple(A2), 10, (0, 255, 255), -1)  # A2
        cv2.circle(result_image, tuple(B2), 10, (0, 255, 255), -1)  # B2

    # Visual debugging: Display detected contours
    debug_image = image.copy()
    cv2.drawContours(debug_image, contours, -1, (0, 255, 0), 1)

    # Display results
    cv2.imshow("Contours detectes", debug_image)
    cv2.imshow("Resultat avec limite ajustee", result_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Test with provided images
detect_and_draw_path("images/camera/WIN_20241121_09_58_45_Pro.jpg")



