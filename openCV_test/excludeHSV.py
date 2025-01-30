import cv2
import numpy as np

def exclude_hsv_colors(image, exclude_ranges, include_ranges):
    """
    Exclude specific HSV color ranges while keeping desired ones.

    Parameters:
        image (ndarray): The input image in BGR format.
        exclude_ranges (list of tuples): List of HSV ranges to exclude (min, max).
        include_ranges (list of tuples): List of HSV ranges to include (min, max).

    Returns:
        mask (ndarray): Final binary mask with excluded and included regions.
    """
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Initialize masks
    exclude_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    include_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)

    # Exclude ranges
    for (lower, upper) in exclude_ranges:
        temp_mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
        exclude_mask = cv2.bitwise_or(exclude_mask, temp_mask)

    # Include ranges
    for (lower, upper) in include_ranges:
        temp_mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
        include_mask = cv2.bitwise_or(include_mask, temp_mask)

    # Combine masks: Keep only included regions that are not excluded
    final_mask = cv2.bitwise_and(include_mask, cv2.bitwise_not(exclude_mask))

    return final_mask


def main():
    # Load the image
    image_path = "images/camera/1_2MP/stpi_gei/WIN_20241129_10_40_56_Pro.jpg"  # Replace with your image path
    image = cv2.imread(image_path)

    if image is None:
        print("Error loading image.")
        return

    # Define ranges to exclude (artefacts)
    exclude_ranges = [
        ([0, 0, 155], [0, 0, 190]),        # Grayscale regions
        ([35, 0, 150], [40, 20, 185]),     # Light greenish
        ([120, 0, 150], [130, 20, 180]),   # Slightly purple
        ([100, 0, 120], [120, 30, 200]),   # Bluish artifacts
        ([10, 0, 180], [30, 30, 200]),     # Yellowish tones
        ([20, 0, 150], [40, 40, 210]),     # Low-intensity yellow-green
        ([110, 0, 80], [120, 150, 130]),   # Dark green artifacts
    ]

    # Define ranges to include (desired features)
    include_ranges = [
        ([100, 20, 200], [110, 50, 255]),  # Bluish-green path
        ([110, 100, 50], [120, 140, 80]),  # Strong green
        ([100, 5, 200], [130, 40, 255]),   # Desired bluish-green
        ([110, 120, 40], [130, 150, 80]),  # Strong blue tones
    ]

    # Apply HSV exclusion and inclusion
    mask = exclude_hsv_colors(image, exclude_ranges, include_ranges)

    # Apply mask to the original image
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the result
    cv2.imshow("Original Image", image)
    cv2.imshow("Filtered Image", result)
    cv2.imshow("Mask", mask)

    # Wait for key press and close windows
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
