import cv2

# Define the camera and open it (usually /dev/video0)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set up video writer parameters
# Define the codec and create VideoWriter object
# 'XVID' is a common codec, but you can try others like 'MJPG' if this doesn't work
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_filename = "captured_video.avi"
out = cv2.VideoWriter(video_filename, fourcc, 20.0, (640, 480))

# Capture 5 seconds of video
frame_count = 0
frame_limit = 20 * 5  # 20 frames per second for 5 seconds

while frame_count < frame_limit:
    ret, frame = cap.read()  # Capture frame-by-frame
    
    if not ret:
        print("Error: Could not read frame.")
        break

    # Write the frame to the video file
    out.write(frame)

    # Increment frame count
    frame_count += 1

    # Press 'q' to stop recording early
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and writer, and close windows
cap.release()
out.release()
cv2.destroyAllWindows()

print("Video saved as {}".format(video_filename))