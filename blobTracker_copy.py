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

# Function to check if two blobs can be merged
def mergeable(blob1, blob2):
    if (blob1.top < blob2.bottom and blob1.top > blob2.top and blob1.left < blob2.right and blob1.left > blob2.left):  # topleft
        return True
    elif (blob1.bottom > blob2.top and blob1.bottom < blob2.bottom and blob1.left < blob2.right and blob1.left > blob2.left):  # bottomleft
        return True
    elif (blob1.top < blob2.bottom and blob1.top > blob2.top and blob1.right > blob2.left and blob1.right < blob2.right):  # topright
        return True
    elif (blob1.bottom > blob2.top and blob1.bottom < blob2.bottom and blob1.right > blob2.left and blob1.right < blob2.right):  # bottomright
        return True
    else:
        return False

# Function to merge two blobs
def merge(blob1, blob2):
    blob = Blob()
    blob.left = min(blob1.left, blob2.left)
    blob.right = max(blob1.right, blob2.right)
    blob.bottom = max(blob1.bottom, blob2.bottom)
    blob.top = min(blob1.top, blob2.top)
    blob.x = (blob1.left + blob1.right) / 2
    blob.y = (blob1.top + blob1.bottom) / 2
    blob.area = (blob1.right - blob1.left) * (blob1.bottom - blob1.top)
    return blob
# Function to merge all blobs in a list
def mergeBlobs(blobs):
    canMerge = True
    while canMerge and len(blobs) > 1:
        finalBlob = blobs[0]
        newBlobs = []
        canMerge = False
        for i in range(1, len(blobs)):
            currBlob = blobs[i]
            if mergeable(finalBlob, currBlob):
                finalBlob = merge(finalBlob, currBlob)
                canMerge = True
            else:
                newBlobs.append(currBlob)
        newBlobs.append(finalBlob)
        blobs = newBlobs
    return blobs

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
    blue = []
    setofblob = []
    for blob in blobs:
        if blob.name == "NeonPink":
	    pink.append(blob)
	if blob.name == "Blue":
	    blue.append(blob)
    setofblob.append(pink)
    setofblob.append(blue)
    return setofblob


def inside(pink, blue):
    for b in blue:
        for p in pink:
	    if (p.right < b.right and p.right > b.left and
		p.left > b.left and p.left < b.right and
		p.top > b.top and p.top < b.bottom and
		p.bottom < b.bottom and p.bottom > b.top):
			return p


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
        sortedblobs = splitbycolor(blobsCopy.blobs)
        blue = mergeBlobs(sortedblobs.pop())
        pink = mergeBlobs(sortedblobs.pop())

        for b in blue:
            cv2.rectangle(color_image, (b.left, b.top), (b.right, b.bottom), (0, 255, 0), 2)
        for b in pink:
	        cv2.rectangle(color_image, (b.left, b.top), (b.right, b.bottom), (0, 255, 0), 2)
        cv2.imshow("Blob Locale", color_image)
        cv2.waitKey(1)
	
	
	    b = inside(pink, blue)
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
