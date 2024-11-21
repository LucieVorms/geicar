import cv2

# Open the camera (usually /dev/video0)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Capture a single frame
ret, frame = cap.read()

# Check if the frame was captured successfully
if not ret:
    print("Error: Could not read frame.")
    cap.release()
    exit()

# Save the captured frame to a file
image_filename = "captured_image.jpg"
cv2.imwrite(image_filename, frame)
print("Image saved as {}".format(image_filename))

# Release the camera
cap.release()