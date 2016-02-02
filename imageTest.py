import numpy
import yarp
import matplotlib.pylab
import time
import sys
import find_lines
 
# Initialise YARP
yarp.Network.init()

# Create a port and connect it to the iCub simulator virtual camera
input_port = yarp.Port()
input_port.open("/python-image-port")
yarp.Network.connect("/iCub/camCalib/out", "/python-image-port")
 
width = 640
height = 480 
# Create numpy array to receive the image and the YARP image wrapped around it
img_array = numpy.zeros((height, width, 3), dtype=numpy.uint8)
yarp_image = yarp.ImageRgb()
yarp_image.resize(width, height)
yarp_image.setExternal(img_array, img_array.shape[1], img_array.shape[0])
#yarp_image.setExternal(img_array.__array_interface__['data'][0], img_array.shape[1], img_array.shape[0]) 




input_port.read(yarp_image)

#for x in range(width):
#  for y in range(height):
#    pix = yarp_image.pixel(x,y)
#    img_array[y,x] = [pix.r, pix.g, pix.b]
# there is yarp_image.getIplImage()
  

# Read the data from the port into the image


#matplotlib.pylab.imshow([[1,0],[0,1]])
#matplotlib.pylab.show()
# display the image that has been read
#matplotlib.pylab.imshow(img_array)
#matplotlib.pylab.show()
img_bgr = img_array[:,:,[2,1,0]]
#find_lines.run_system(img_bgr)
print time.time()
matplotlib.image.imsave('test'+sys.argv[1]+'.tiff', img_array, format='tiff')
print time.time()
# Cleanup
input_port.close()
