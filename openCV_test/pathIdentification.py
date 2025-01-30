import cv2
import os
import numpy as np


def detect_edges(image):
    # Convert the image to greyscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Apply a blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)
    # Use the Canny detector to detect edges
    edges = cv2.Canny(blurred, threshold1=50, threshold2=200)
    cv2.imshow("edges", edges)
    return edges


def find_contours(edges):
    # Find contours from detected edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def detect_and_draw_path(image):
    # Image pre-processing
    edges = detect_edges(image)

    contours = find_contours(edges)

    height, width = edges.shape
    center_x = width // 2

    # Draw a vertical line through the centre of the image for reference
    cv2.line(image, (center_x, 0), (center_x, height), (255, 0, 0), 2)

    # Divide the image into 4 parts: half the width, 3/4 and 1/4 the height
    half_y = 3 * height // 4
    exclude_band_y = int(height * 1)  # Exclude the bottom 13% of the image

    # Categorise contours by part of the image, excluding the bottom band
    left_contours = [
        pt for contour in contours for pt in contour 
        if pt[0][0] < center_x and half_y <= pt[0][1] < exclude_band_y
    ]
    right_contours = [
        pt for contour in contours for pt in contour 
        if pt[0][0] >= center_x and half_y <= pt[0][1] < exclude_band_y
    ]

    # Initialise the points to draw the line to delimit the right and left sides of the path
    A1, A2, B1, B2 = None, None, None, None

    # Find the points to draw the line to delimit the right-hand side of the path
    if right_contours:
        A1 = min(right_contours, key=lambda pt: (pt[0][1], pt[0][0]))[0]  # Top + right
        B1 = max(right_contours, key=lambda pt: pt[0][1])[0]  # Bottom + right

    # Find the points to draw the line to delimit the left-hand side of the path
    if left_contours:
        A2 = min(left_contours, key=lambda pt: (pt[0][1], -pt[0][0]))[0]  # Top + left
        B2 = max(left_contours, key=lambda pt: pt[0][1])[0]  # Bottom + left

    # Align the heights of A1 and A2, B1 and B2
    if A1 is not None and A2 is not None:
        avg_top_y = (A1[1] + A2[1]) // 2
        A1[1] = avg_top_y
        A2[1] = avg_top_y

    if B1 is not None and B2 is not None:
        avg_bottom_y = (B1[1] + B2[1]) // 2
        B1[1] = avg_bottom_y
        B2[1] = avg_bottom_y

    # Draw lines and points
    path_center_x = None
    if A1 is not None and B1 is not None:
        cv2.line(image, tuple(A1), tuple(B1), (255, 0, 0), 3)  # Blue line (right)
        cv2.circle(image, tuple(A1), 10, (255, 255, 0), -1)  # A1
        cv2.circle(image, tuple(B1), 10, (255, 255, 0), -1)  # B1
    if A2 is not None and B2 is not None:
        cv2.line(image, tuple(A2), tuple(B2), (0, 0, 255), 3)  # Red line (left)
        cv2.circle(image, tuple(A2), 10, (0, 255, 255), -1)  # A2
        cv2.circle(image, tuple(B2), 10, (0, 255, 255), -1)  # B2

    # Compute the middle of the path if detected
    if A1 is not None and B1 is not None and A2 is not None and B2 is not None:
        # Compute mean position of the lower points
        path_center_x = (B1[0] + B2[0]) // 2
        path_center_y = (B1[1] + B2[1]) // 2
        path_center = (path_center_x, path_center_y)
        cv2.circle(image, path_center, 5, (0, 255, 0), -1)  # Centre du chemin

    return image, path_center_x, center_x, A1, B1, A2, B2


def adjust_direction(path_center_x, center_x):
    # Ajust direction 
    if path_center_x is not None:
        if path_center_x < center_x - 20:
            print("Turn left")
        elif path_center_x > center_x + 20:
            print("Turn right")
        else:
            print("Go forward")
    else:
        print("Path not detected")


def main():
    # Directory with all images
    dossier_images = "images/camera/1280_720"

    # Get all images in that directory
    image_paths = [
        os.path.join(dossier_images, f)
        for f in os.listdir(dossier_images)
        if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tiff'))
    ]

    for image_path in image_paths:
        # Read the image
        full_frame = cv2.imread(image_path)

        if full_frame is None:
            print(f"Erreur: Impossible de charger l'image {image_path}.")
            continue

        # Make a copy to display
        display_frame = full_frame.copy()

        # Detect and draw the path
        display_frame, path_center_x, center_x, pA1, pB1, pA2, pB2 = detect_and_draw_path(display_frame)
        adjust_direction(path_center_x, center_x)

        # Display the image with contours and the path delimitation
        cv2.imshow("Contours du chemin", display_frame)

        # Wait action from the user to change image
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

    # Close windows
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
