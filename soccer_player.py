#!/usr/bin/python

import rospy, cv2, copy, math
from tf.transformations import euler_from_quaternion
from cmvision.msg import Blobs, Blob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Publisher for movement
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)

firstBallMarked =False
firstBallSeen=False
firstGoalSeen=False
firstGoalMarked=False
moved = False


colorImage = Image() # image object used for display, from camera callback
blobsInfo = Blobs() # 'blobs' object we get from blobcallback
isColorImageReady = False # Want both of these before we continue
isBlobsInfoReady = False
odom = Odometry()
isOdomReady = False
old_error = None
twist = Twist()



# Callback form camera
def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

# Clabback for odometry
def updateOdom(data):
    global odometry, isOdomReady
    odomerty = data
    isOdomReady = True

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

def splitByColor(blobs):
    ballColor = "Ball"
    innerGoalColor = "InnerGoal"
    outerGoalColor = "OuterGoal"
    bigColor=[]
    ball=[]
    innerGoal=[]
    outerGoal = []
    
    for b in blobs:
        if b.name == ballColor:
            ball.append(b)
        elif b.name == outerGoalColor:
            outerGoal.append(b)
        elif b.name == innerGoalColor:
            innerGoal.append(b)
    bigColor.append(ball)
    bigColor.append(innerGoal)
    bigColor.append(outerGoal)
    return bigColor
    

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
                    blobs.pop(n) # get rid of merged bloib
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
            if b.area > largestBlob.area:
                largestBlob = b
        return largestBlob
    else:
        return None

# returns largest blob inside another blob
def theBlobWithin(inner, outer):
    largestBlob = Blob()
    largestBlob.area = 0
    for b in outer:
        for p in inner:
            if (p.right < b.right and p.right > b.left and
            p.left > b.left and p.left < b.right and
            p.top > b.top and p.top < b.bottom and
            p.bottom < b.bottom and p.bottom > b.top
            and p.area > largestBlob.area):
                # if p within b AND p larger than last found p
                largestBlob = p
    if largestBlob.area > 0:
            return largestBlob
    else:
        return None



# TODO test and adjust
# with given blob, tries to get blob to center of camera by turning towards it
# 
def honeInOnBlob(ball):
    global pub, rate,old_error, twist
    PROPORTIONAL_VAR = .01
    DIFFERENTIAL_VAR = .004
    if (ball != None):
        error = 320 - ball.x
        if old_error == None:
            twist.angular.z = PROPORTIONAL_VAR * error
        else:
            differential = error - old_error
            twist.angular.z = (PROPORTIONAL_VAR * error +
                                DIFFERENTIAL_VAR * differential)
        old_error = error
        if abs(error < 2) and differential < 3:
            twist.angular.z = 0
            pub.publish(twist)
            rate.sleep()
            rate.sleep()
            return True
        else:
            print("not quite..")
    else:
	    twist.angular.z = 0
    return False

    

def findBlob(blob):
    global twist
    twist.linear.z = .5
    if blob != None:
        return True
    return False

# odom to degrees. returns degree
def odomToDegree(odom):
    # Convert quaternion to degree
    q = [odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    # roll, pitch, and yaw are in radian
    return yaw * 180 / math.pi


#must set angle back to 0, travel set distance via param
def move(distance):
    global odom
    degree = odomToDegree(odom)
    atZero = False
    if not atZero:
        if degree > 0:
            twist.angular.z = -.2
            atZero = False
        elif degree < 0:
            twist.angular.z = .2
            atZero = False
        if abs(0 - degree) < 2:
            atZero = True
            twist.angular.z = 0
    if atZero:
        x = odom.pose.pose.position.x
        if (x < distance):
            twist.linear.x = .3
        if x > distance * .75:
            twist.linear.x = .2
        if abs(distance - x) < .05:
            twist.linear.x = 0
            return True
    return False
    

# main loop
def main():
    rospy.init_node('soccer_blob_tracker')
    rospy.Subscriber('/v4l/camera/image_raw', Image, updateColorImage)
    rospy.Subscriber('/blobs', Blobs, updateBlobsInfo)
    rospy.Subscriber('/odom', Odometry, updateOdom)
    bridge = CvBridge()

    firstBallMarked =False
    secondBallMarked = False
    firstBallSeen=False
    secondBallSeen = False
    firstGoalSeen=False
    secondGoalSeen = False
    firstGoalMarked=False
    secondGoalMarked = False
    moved = False    
    global ball,twist, rate, goal, old_error, odom, isOdomReady
    firstBallOdom=None
    firstGoalOdom=None
    secondBallOdom=None
    secondGoalOdom = None
    rate = rospy.Rate(10)  # 10 Hz
    # hol up. do we have everything?
    while not rospy.is_shutdown() and(not isColorImageReady or not isBlobsInfoReady or not isOdomReady):
        pass

    # beefcakes
    while not rospy.is_shutdown():
        
        try:
            cv_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError as e:
            print(e)
            continue

        # a lot here. big picture is get ball blob and goal blob
        bigBlob = splitByColor(filterBlobs(blobsInfo.blobs))
        unmergedOuterGoal = bigBlob.pop()
        UMinnerGoal = bigBlob.pop()
        UMball = bigBlob.pop()
        innerGoalList = mergeAllBlobs(UMinnerGoal)
        outerGoalList = mergeAllBlobs(unmergedOuterGoal)
        ballList = mergeAllBlobs(UMball)
        ball = getLargestBlob(ballList)
        goal = theBlobWithin(innerGoalList, outerGoalList)


        # start of main logic
        # we need to make sure that ^^^^ code runs every cycle. 
        # no loops within methods, use big while here to iterate. 
        # recall with new blobs
        if not firstBallSeen:
            firstBallSeen = findBlob(ball)
        elif not firstBallMarked:
            firstBallMarked = honeInOnBlob(ball)
            if firstBallMarked:
                firstBallOdom = odom
                old_error = None
                print("foundfirstball")
        elif not firstGoalSeen:
            firstGoalSeen = findBlob(goal)
        elif not firstGoalMarked:
            firstGoalMarked = honeInOnBlob(goal)
            # when we find blob, also reset old_error
            #TODO this record is bad and happens every loop....
            #  maybe in method stop and return odom?
            if firstGoalMarked:
                firstGoalOdom = odom
                old_error = None
                print("foundfirstgoal")
        elif not moved:
            moved = move()
        elif not secondBallSeen:
            secondBallSeen = findBlob()
        elif not secondBallMarked:
            secondBallMarked = honeInOnBlob(ball)
            if secondBallMarked:
                secondBallOdom = odom
                old_error = None
                print("foundsecondball")
        elif not secondGoalSeen:
            secondGoalSeen = findBlob(goal)
        elif not secondGoalMarked:
            secondGoalMarked = honeInOnBlob(goal)
            if secondGoalMarked:
                secondGoalOdom = odom
                old_error = None
                print('foundsecondgoal')

        # INSERT MATH


        pub.publish(twist)

        # Display the image with the blobs
        if ball != None:
	        cv2.rectangle(cv_image, (ball.left, ball.top), (ball.right, ball.bottom), (0, 255, 0), 2)
        if goal != None:
	        cv2.rectangle(cv_image, (goal.left, goal.top), (goal.right, goal.bottom), (0, 255, 0), 2)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        rate.sleep()



if __name__ == '__main__':
        main()
