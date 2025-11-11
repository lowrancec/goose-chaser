import cv2
print(cv2.__version__)

import PySpin
import cv2
import os
import numpy as np
from datetime import datetime
from multiprocessing.pool import ThreadPool
from collections import deque

# This function is called by multiple threads
def save_img(image_data, fname):
	cv2.imwrite(fname, image_data, [cv2.IMWRITE_PNG_COMPRESSION, 1])
	
# Create a directory to put images into
data_dir = "Data"
os.makedirs(data_dir, exist_ok=True)
print(f"Directory '{data_dir}' ensured to exist.")

# Saving to disk with threads otherwise lag is long
threadn = cv2.getNumberOfCPUs()
pool = ThreadPool(processes = threadn)
pending = deque()

# Setting up the camera 
system = PySpin.System.GetInstance()
cam_list = system.GetCameras()
cam = cam_list.GetByIndex(0) # Get the first camera
cam.Init()
ser_num = cam.TLDevice.DeviceSerialNumber.GetValue()
cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
cam.BeginAcquisition()    


try:
	while True:
		# Get next image from camera
		image_result = cam.GetNextImage(1000) # Timeout in milliseconds

		if image_result.IsIncomplete():
			print("Image incomplete with error %d" % image_result.GetImageStatus())
		else:
			# Convert PySpin image to OpenCV format (NumPy array)
			# Ensure correct data type and reshaping based on pixel format
			# Example for Mono8:
			image_data = image_result.GetData().reshape(image_result.GetHeight(), image_result.GetWidth())
			
			# Get Time and use it in filename
			tNow = datetime.now() 
			dtime_data = datetime.now().strftime('%Y%m%d_%H%M%S_%f')#:-3]	
			image_filename = data_dir + "/Chameleon" + ser_num + "_" + dtime_data + ".png"
						
			# For color images (e.g., BayerRG8), you might need to convert using cv2.cvtColor
			image_data = cv2.cvtColor(image_data, cv2.COLOR_BayerGR2BGR)

			# Display the frame using OpenCV
			cv2.imshow("Spinnaker Camera Feed", image_data)
			 
			# Save the image to file using multiple threads	
			while len(pending) > 0 and pending[0].ready():
			    t0 = pending.popleft()        
			if len(pending) < threadn:
			    task = pool.apply_async(save_img, (image_data.copy(), image_filename))
			    pending.append(task)
			
			# Release the PySpin image
			image_result.Release()

		# Break the loop if 'q' is pressed
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

except PySpin.SpinnakerException as ex:
	print("Error: %s" % ex)    
	
# Clean up
cam.EndAcquisition()
cam.DeInit()
del cam
cam_list.Clear()
system.ReleaseInstance()
cv2.destroyAllWindows()    
    
    
    
    
    
    
