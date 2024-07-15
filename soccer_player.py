#!usr/bin/python

import rospy, cv2, copy, math
from tf.transformations import euler_from_quaternion
from cmvision.msg import Blobs, Blob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from blob_tracker2 import updateColorImage, updateBlobsInfo, mergeAllBlobs, splitByColor, getLargestBlob ###is it block tracker2 idk

# Publisher for movement
pub = rospy.Publisher('/mobile_base/commans/velocity', Twist, queue_size = 1)

firstBallMarked, firstBallSeen, firstGoalSeen, firstGoalMarked, moved = False


colorImage = Image() # image object used for display, from camera callback
blobsInfo = Blobs() # 'blobs' object we get from blobcallback
isColorImageReady = False # Want both of these before we continue
isBlobsInfoReady = False
odom = Odometry()
isOdomReady = False





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

def filterBlobs(blobs):
    ballColor = "ENTERCOLORNAME"
    innerGoalColor = "ENTERINNERGOAL"
    outerGoalColor = "ENTEROUTERGOAL"
    bigColor, ball, innerGoal, outerGoal = []
    
    for b in blobs:
        if b.color == ballColor:
            ball.append(b)
        elif b.color == outerGoalColor:
            outerGoal.append(b)
        elif b.color == innerGoalColor:
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
    old_error = 0
    PROPORTIONAL_VAR = .005
    DIFFERENTIAL_VAR = .04
    ballFound = False
    speed = 0
    if (ball != None):
        error = 320 - ball.x
        differential = error - old_error
        old_error = error
        speed = (PROPORTIONAL_VAR * error +
                            DIFFERENTIAL_VAR * differential)
    else:
        speed = 0
    if (abs(error < 2)):
        speed = 0
        ballFound = True

    return speed

    



# finds first ball. Returns odom angle
def findFirstBall(ball):
    global firstBallSeen
    speed = .1
    if (ball != None):
            firstBallSeen = True
    return speed
    ####MUUUUUST GET ACCURATE MEASUREMENT 


def findFirstGoal(goal):
    global firstGoalSeen
    speed = .1
    if ball != None:
        firstGoalSeen = True
    return speed

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





# main loop
def main():
    rospy.init_node('soccer_blob_tracker')
    rospy.Subscriber('/camera/rgb/image_raw', Image, updateColorImage)
    rospy.Subscriber('/blobs', Blobs, updateBlobsInfo)
    rospy.subscriber('/odom', Odometry, updateOdom)
    bridge = CvBridge()
    rate = rospy.Rate(10)  # 10 Hz
    global firstBallMarked, firstBallSeen, firstGoalSeen, firstGoalMarked
    global ball, goal, odom, isOdomReady
    firstBallOdom, firstGoalOdom, secondBallOdom, secondGoalOdom = None
    ballTriangle, goalTriangle = triangle()
    old_error = 0
    # hol up. do we have everything?
    while not isColorImageReady and not isBlobsInfoReady and not isOdomReady:
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

        twist = Twist()

        # start of main logic
        # we need to make sure that ^^^^ code runs every cycle. 
        # no loops within methods, use big while here to iterate. 
        # recall with new blobs
        if not firstBallSeen:
            findFirstBall(ball, goal)
        elif not firstBallMarked:
            twist.angular.z = honeInOnBlob(ball)
            firstBallOdom = odom
        elif not firstGoalSeen:
            findFirstGoal(goal)
        elif not firstGoalMarked:
            twist.angular.z = honeInOnBlob(goal)
            # when we find blob, also reset old_error
            #TODO this record is bad and happens every loop....
            #  maybe in method stop and return odom?
            firstGoalOdom = odom
        elif not moved:
            move()
        elif not secondBallSeen:
            findSecondBall()
        elif not secondBallFound:
            twist.angular.z = honeInOnBlob(ball)
            # RECORD BETTER
            secondBallOdom = odom
        elif not secondGoalSeen:
            findSecondGoal()
        elif not secondGoalMarked:
            twist.angular.z = honeInOnBlob(goal)
            # RECORD
            secondGoalOdom = odom

    


        
        
        
        
        



        pub.publish(twist)

        # Display the image with the blobs
        cv2.rectangle(cv_image, (ball.left, ball.top), (ball.right, ball.bottom), (0, 255, 0), 2)
        cv2.rectangle(cv_image, (goal.left, goal.top), (goal.right, goal.bottom), (0, 255, 0), 2)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except:
        pass