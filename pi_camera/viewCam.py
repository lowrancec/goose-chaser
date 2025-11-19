from picamera2 import Picamera2
import cv2

# List cameras (optional, for confirmation)
print("Available cameras:", Picamera2.global_camera_info())

# Create the camera instance — use the first CSI camera
picam2 = Picamera2(0)

# Configure a preview (match native resolution of the IMX296)
config = picam2.create_preview_configuration(main={"size": (1456, 1088), "format": "XRGB8888"})
picam2.configure(config)
picam2.start()

print("Press 'q' to quit.")
while True:
    frame = picam2.capture_array()
    cv2.imshow("Global Shutter Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.close()
