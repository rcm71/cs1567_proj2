#!usr/bin/python

import rospy, cv2, copy
from cmvision.msg import Blobs, Blob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

# Publisher for movement
pub = rospy.Publisher('/mobile_base/commans/velocity', Twist, queue_size = 1)

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
    while len(blobs) > 1: # while we have work to do
        superBlob = blobs[0] # Grab first in list to use to merge
        canMerge = True
        while canMerge: # keep merging if we have made a merge
            canMerge = False
            for n in range(1, len(blobs)): # Through all blobs
                if mergable(superBlob, blobs[n]): # checks if corners overlap
                    canMerge = True
                    superBlob = mergeBlobs(superBlob, blobs[n])# actual merge
                    blobs.pop(n) # get rid of merged blob
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




# Main method, uses while loop to keep robot working
def main():
    global pub, colorImage, blobsInfo, isColorImageReady, isBlobsInfoReady
    rospy.init_node('blobtracker', anonymous = True) # our node
    rospy.Subscriber('/v4l/camera/image_raw', Image, updateColorImage) # image channel
    rospy.Subscriber('/blobs', Blobs, updateBlobsInfo) # blobs channel gives blobs
    bridge = CvBridge() # Used for coloring image
    cv2.namedWindow('Blob Locale') # Display window
    twist = Twist() # direction command for robot

    # waiting on our image and blobs to be ready
    while not rospy.is_shutdown() and (not isBlobsInfoReady or not isColorImageReady):
        pass

    while not rospy.is_shutdown():
        try:
            recolored_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except: CvBridgeError as e:
            print(e)
            continue
    filteredBlobs = filterBlobs(blobsInfo.blobs)
    mergedBlobs = mergeAllBlobs(filteredBlobs)

if __name__ == '__main__':
    try:
        main()
    except:
        pass

