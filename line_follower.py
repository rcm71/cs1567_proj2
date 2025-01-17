#!/usr/bin/python

import rospy, cv2, copy, math
from cmvision.msg import Blobs, Blob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


# Publisher for movement
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)

colorImage = Image() # image object used for display, from camera callback
blobsInfo = Blobs() # 'blobs' object we get from blobcallback
isColorImageReady = False # Want both of these before we continue
isBlobsInfoReady = False

# Callback form camera
def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

# Callback to get blobs
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


# Filters out small blobs
def filterBlobs(blobs):
    bigBlobs = []
    for b in blobs:
        if b.area > 20:
            bigBlobs.append(b)
    return bigBlobs

# returns largest blob in list
def getLargestBlob(blobs):
    if len(blobs) > 0:
        largestBlob = blobs[0]
        for b in blobs:
            if b.area > largestBlob:
                largestBlob = b
        return largestBlob
    else:
        return None

# Main method, uses while loop to keep robot working
def main():
    global pub, colorImage, blobsInfo, isColorImageReady, isBlobsInfoReady
    old_error = 0
    integral = 0    
    rospy.init_node('linefollower', anonymous = True) # our node
    rospy.Subscriber('/v4l/camera/image_raw', Image, updateColorImage) # image channel
    rospy.Subscriber('/blobs', Blobs, updateBlobsInfo) # blobs channel gives blobs
    bridge = CvBridge() # Used for coloring image
    cv2.namedWindow('Blob Locale') # Display window
    twist = Twist() # direction command for robot
    rate = rospy.Rate(100)
    # waiting on our image and blobs to be ready
    while not rospy.is_shutdown() and (not isBlobsInfoReady or not isColorImageReady):
        pass

    while not rospy.is_shutdown():
        try:
            recolored_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError as e:
            print(e)
            continue
    
        # FIlter, merge, get largest blob
        filteredBlobs = filterBlobs(blobsInfo.blobs)
        mergedBlobs = mergeAllBlobs(filteredBlobs)
        largestBlob = getLargestBlob(mergedBlobs)
        
        # calulus time! see PID slides
        # this is how we adjust our turning
        PROPORTIONAL_VAR = .005 # adjust until oscillation begins
        DIFFERENTIAL_VAR = .04 # then adjust until oscillation stops
        INTEGRAL_VAR = 0 # final adjustments
        if (largestBlob != None):
            error = 320 - largestBlob.x
            differential = error - old_error
            integral += error # theres no way this is right been awhile since calc 2
            old_error = error
	    if abs(error) < 10:
		    twist.angular.z = 0
	    else:
	        twist.angular.z = (PROPORTIONAL_VAR * error +
                            INTEGRAL_VAR * integral +
                            DIFFERENTIAL_VAR * differential)
            twist.linear.x = .2
	    print(largestBlob.x)
	    else:
            twist.angular.z = 0
	    twist.linear.x = 0
            print("no blob!")
	pub.publish(twist)
	rate.sleep()
    # end while
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
