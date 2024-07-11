#!/usr/bin/python

import rospy, cv2, copy
from cmvision.msg import Blobs, Blob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

# Publisher for robot velocity commands
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

colorImage = Image()
isColorImageReady = False
blobsInfo = Blobs()
isBlobsInfoReady = False

# Callback to update the color image
def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

# Callback to update the blobs information
def updateBlobsInfo(data):
    global blobsInfo, isBlobsInfoReady
    blobsInfo = data
    isBlobsInfoReady = True

# Boolean check on whether blobs overlap
def mergable(a, b):
    mergable = False
    
    # Need to check inverse (top right = bottom left)
    # of seconary blob against a's top left and bottom right 
    if (a.left < b.right and b.right < a.right and
        a.top < b.bottom and b.bottom < a.bottom):
        mergable = True # TOP LEFT
    elif(a.left < b.right and b.right < a.right and
        a.top < b.top and b.top < a.bottom):
        mergable = True # BOTTOM LEFT
    elif(a.left < b.left and b.left < a.right and
         a.top < b.top and b.top < a.bottom):
        mergable = True # BOTTOM RIGHT
    elif(a.left < b.left and b.left < a.right and
         a.top < b.bottom and b.bottom < a.bottom):
        mergable = True # TOP RIGHT

    return mergable

# Merges two blobs into max size
def mergeBlobs(a, b):
    superBlob = Blob()
    superBlob.name = a.name
    superBlob.red = a.red
    superBlob.green = a.green
    superBlob.blue = a.blue
    superBlob.left = min(a.left, b.left)
    superBlob.right = max(a.right, b.right)
    superBlob.top = min(a.top, b.top)
    superBlob.bottom = max(a.bottom, b.bottom)
    superBlob.x = superBlob.left + superBlob.right / 2
    superBlob.y = superBlob.top + superBlob.bottom / 2
    superBlob.area = ((superBlob.bottom - superBlob.top)*
                      (superBlob.right - superBlob.bottom))
    return superBlob

# Merges all touching blobs
def mergeAllBlobs(blobs):
    mergedBlobs = [] # final blobs list
    if len(blobs) == 1:
	return blobs

    while len(blobs) > 1: # while we have work to do
        superBlob = blobs[0] # Grab first in list to use to merge
        canMerge = True
        while canMerge: # keep merging if we have made a merge
            canMerge = False
	    n = 1
            while n < len(blobs): # Through all blobs
                if mergable(superBlob, blobs[n]): # checks if corners overlap
                    canMerge = True
                    superBlob = mergeBlobs(superBlob, blobs[n])# actual merge
                    blobs.pop(n) # get rid of merged blob
		n += 1
        mergedBlobs.append(superBlob) # canMerge = false
        blobs.pop(0) # get rid of superblob
    return mergedBlobs

# Function to find the largest blob
def biggestBlob(blobs):
    big = Blob()
    big.area = 0
    for blob in blobs:
        if blob.area > big.area:
            big = blob
    return big

def splitbycolor(blobs):
    pink = []
    orange = []
    setofblob = []
    for blob in blobs:
        if blob.name == "NeonPink":
	    pink.append(blob)
	if blob.name == "Orange":
	    orange.append(blob)
    setofblob.append(pink)
    setofblob.append(orange)
    return setofblob

# pink inside blue
def inside(pink, blue):
    for b in blue:
        for p in pink:
	    if (p.right < b.right and p.right > b.left and
		p.left > b.left and p.left < b.right and
		p.top > b.top and p.top < b.bottom and
		p.bottom < b.bottom and p.bottom > b.top):
			return p

# Filters out small blobs
def filterBlobs(blobs):
    bigBlobs = []
    for b in blobs:
        if b.area > 20:
            bigBlobs.append(b)
    return bigBlobs

def main():
    global colorImage, isColorImageReady, blobsInfo, isBlobsInfoReady
    rospy.init_node('blobtracker', anonymous=True)
    rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
    rospy.Subscriber('/blobs', Blobs, updateBlobsInfo)
    bridge = CvBridge()
    cv2.namedWindow("Blob Locale")
    twist = Twist()

    while not rospy.is_shutdown() and (not isBlobsInfoReady or not isColorImageReady):
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError as e:
            print(e)
            continue

        blobsCopy = copy.deepcopy(blobsInfo)
        filteredBlobs = filterBlobs(blobsCopy.blobs)
	sortedblobs = splitbycolor(filteredBlobs)
        orange = mergeAllBlobs(sortedblobs.pop())
        pink = mergeAllBlobs(sortedblobs.pop())

        for b in orange:
            cv2.rectangle(color_image, (b.left, b.top), (b.right, b.bottom), (0, 255, 0), 2)
        for b in pink:
	    cv2.rectangle(color_image, (b.left, b.top), (b.right, b.bottom), (0, 255, 0), 2)
        cv2.imshow("Blob Locale", color_image)
        cv2.waitKey(1)
	
	
	b = inside(pink, orange)
        if b != None: 
	    if b.x > 325:
                twist.angular.z = -0.5
            elif b.x < 315:
                twist.angular.z = 0.5
            else:
                twist.angular.z = 0.0

        pub.publish(twist)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
