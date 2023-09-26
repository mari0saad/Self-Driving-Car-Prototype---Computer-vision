/*Mario*/
import cv2
import numpy as np
import time
import ast
import imutils
'''
#stor the data
file_name = 'values.txt'
signal = []
errors = []
speed = []
time_ms = []
'''
# read video
# "project_video.mp4"


speed = 15  #normal speed
##################################
#parameters for sign
detection_interval = 14  # seconds
number_of_reads = 1
first_detection = True
last_detection_time = time.time()

#first action is left
left_interval = 5
last_left_time = time.time()
left_flag = False


##############################
# for PID
dedt =1.1
eintegral =1.1
eprev =1.1
u= 1.1
delta_t = 1.1
# error =0
first = True

# open the camera
video = cv2.VideoCapture(0)
print(video.get(cv2.CAP_PROP_FRAME_WIDTH))
print(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

while True:
    ret, orig_frame = video.read()


    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    if first:
        l1 = 240
        l2 = 240
        pl1 = 240
        pl2 = 240
        error = 0
        t = time.time()
    else:
        pl1 = l1
        pl2 = l2
        pt = t
        t = time.time()
        delta_t = t - pt
    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    gray = cv2.cvtColor(orig_frame, cv2.COLOR_BGR2GRAY)
    Blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(Blur, 75, 150)

    # region of interest for lane
    mask = np.zeros_like(edges)
    rectangle = np.array([[(20, 400), (620, 400), (640, 470), (0, 470)]], np.int32)
    cv2.fillPoly(mask, rectangle, 255)
    masked = cv2.bitwise_and(mask, edges)

    # detect all lines in this region
    lines = cv2.HoughLinesP(masked, 1, np.pi / 180, 50, maxLineGap=200)

    xp1 = []
    yp1 = []
    xp2 = []
    yp2 = []
    xp3 = []
    yp3 = []
    xp4 = []
    yp4 = []
    if lines is not None:
        for line in lines:
            # print(line)
            x1, y1, x2, y2 = line[0]
            # detect which line is right or left
            if x2 < 320:
                xp1.append(x1)
                yp1.append(y1)
                xp2.append(x2)
                yp2.append(y2)
            else:
                xp3.append(x1)
                yp3.append(y1)
                xp4.append(x2)
                yp4.append(y2)
            # cv2.line(orig_frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    # calculate the mean of all lines to find only two lines(left and right)
    lx1 = np.mean(xp1, dtype=np.int32)
    ly1 = np.mean(yp1, dtype=np.int32)
    lx2 = np.mean(xp2, dtype=np.int32)
    ly2 = np.mean(yp2, dtype=np.int32)
    rx1 = np.mean(xp3, dtype=np.int32)
    ry1 = np.mean(yp3, dtype=np.int32)
    rx2 = np.mean(xp4, dtype=np.int32)
    ry2 = np.mean(yp4, dtype=np.int32)

    # draw left and right line
    cv2.line(orig_frame, (lx1, ly1), (lx2, ly2), (0, 255, 255), 5)
    cv2.line(orig_frame, (rx1, ry1), (rx2, ry2), (0, 255, 255), 5)
    line1 = np.array([lx1, ly1, lx2, ly2])
    line2 = np.array([rx1, ry1, rx2, ry2])
    #print(line1)
    #print(line2)

    # center_ref_point = (width_ref,hight_ref)
    width_ref = 320
    hight_ref = 440

    # center_ref_point
    cv2.circle(orig_frame, (width_ref, hight_ref), 5, (0, 255, 0), -1)

    # ref_line
    cv2.line(orig_frame, (0, hight_ref), (1280, hight_ref), (200, 2200, 200), 5)

    if (ly1 != ly2 and ry1 != ry2):

        # calculate x for left and right points (hight is const)
        left_ref_point = int((((hight_ref - ly1) * (lx2 - lx1)) / (ly2 - ly1)) + lx1)
        right_ref_point = int((((hight_ref - ry1) * (rx2 - rx1)) / (ry2 - ry1)) + rx1)

        # draw left and right points
        cv2.circle(orig_frame, (left_ref_point, hight_ref), 5, (0, 255, 0), -1)
        cv2.circle(orig_frame, (right_ref_point, hight_ref), 5, (0, 255, 0), -1)
        l1 = (width_ref - left_ref_point)
        l2 = (right_ref_point - width_ref)
        #print('111111111111111111111111111111111111111111')


    elif ly1 != ly2:

        left_ref_point = int((((hight_ref - ly1) * (lx2 - lx1)) / (ly2 - ly1)) + lx1)
        cv2.circle(orig_frame, (left_ref_point, hight_ref), 5, (0, 255, 0), -1)
        l1 = (width_ref - left_ref_point)
        l2 = 210         # check when the lane is changes
        #print('2222222222222222222222222222222222222222222')

    elif ry1 != ry2:

        right_ref_point = int((((hight_ref - ry1) * (rx2 - rx1)) / (ry2 - ry1)) + rx1)

        # draw left and right points
        cv2.circle(orig_frame, (right_ref_point, hight_ref), 5, (0, 255, 0), -1)
        l1 = 210          # check when the lane is changes
        l2 = (right_ref_point - width_ref)
        #print('33333333333333333333333333333333333333')

    # calculate error
    error = l1 - l2

    print('l1(LHS):', l1)
    print('l2(RHS):', l2)

    print('error is :', error)
    # /////////////////////////(PID)////////////////////

    #  PID
    kp = 8.95692
    ki = 6.9774
    kd = 0.455
    dedt = (error - eprev) / delta_t
    # dedt = (error)/delta_t
    eintegral = eintegral + error * delta_t
    u = kp * error + kd * dedt + ki * eintegral
    #print("error-eprev :", error - eprev)
    eprev = error
    
    if not left_flag:
        # lane action
        #dc.steer(int((u/1600)*60))

    # /////////////////////////////////////////////

    #sign code
    # .....................................................................................

    # region of interest for sign
    mask_s = np.zeros_like(edges)
    rectangle_s = np.array([[(400, 100), (600, 100), (600, 350), (400, 350)]], np.int32)
    cv2.fillPoly(mask_s, rectangle_s, 255)
    masked_s = cv2.bitwise_and(mask_s, edges)

    # detect all circles in this region
    circles = cv2.HoughCircles(masked_s, cv2.HOUGH_GRADIENT, 1, minDist=100, param1=100, param2=50,
                               minRadius=50, maxRadius=70)

    # draw circles
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(orig_frame, (x, y), r, (100, 100, 50), 5)


        if first_detection:
            last_detection_time = time.time()
            first_detection = False

            # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            # take action for signs

            if number_of_reads == 1:
                last_left_time = time.time()
                left_flag = True
                print("First detection!")
            elif number_of_reads == 2:
                
                print("Second detection!")
            elif number_of_reads == 3:
                
                print("Third detection!")
            elif number_of_reads == 4:
                
                print("Fourth detection!")
            elif number_of_reads == 5:
                
                print("Fourth detection!")
            elif number_of_reads == 6:
               
                print("Fourth detection!")
            else :
                number_of_reads = 1

            # end the action of signs
            # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    if not first_detection:
        time_since_last_detection = time.time() - last_detection_time
        if time_since_last_detection >= detection_interval:
            number_of_reads += 1
            last_detection_time = time.time()
            first_detection = True
            
    if left_flag:
        speed = 10 # low speed at turn left
        #send the angle
        time_left = time.time() - last_left_time
        if time_left >= left_interval:
            speed = 15 #the normal speed
            fleft_flag = False


    # ................................................................................

    #color code
    #******************************************************************************************
    hsv = cv2.cvtColor(orig_frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([25, 70, 120])
    upper_yellow = np.array([30, 255, 255])
    lower_green = np.array([40, 70, 80])
    upper_green = np.array([70, 255, 255])
    lower_red = np.array([0, 50, 120])
    upper_red = np.array([10, 255, 255])

    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask2 = cv2.inRange(hsv, lower_green, upper_green)
    mask3 = cv2.inRange(hsv, lower_red, upper_red)

    cnts1 = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts1 = imutils.grab_contours(cnts1)
    cnts2 = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts2 = imutils.grab_contours(cnts2)
    cnts3 = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts3 = imutils.grab_contours(cnts3)

    for c in cnts1:
        area1 = cv2.contourArea(c)
        if area1 > 5000:
            #cv2.drawContours(orig_frame, [c], -1, (255, 0, 0), 3)
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            cv2.circle(orig_frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.putText(orig_frame, 'YellowSign', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (25, 25, 25), 2)
            #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            #action for yellow color

    for c in cnts2:
        area2 = cv2.contourArea(c)
        if area2 > 5000:
            #cv2.drawContours(orig_frame, [c], -1, (255, 0, 0), 3)
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            cv2.circle(orig_frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.putText(orig_frame, 'GreenSign', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (25, 25, 25), 2)
            #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            #action for green color

    for c in cnts3:
        area3 = cv2.contourArea(c)
        if area3 > 5000:
            #cv2.drawContours(orig_frame, [c], -1, (255, 0, 0), 3)
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            cv2.circle(orig_frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.putText(orig_frame, 'RedSign', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (25, 25, 25), 2)
            # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            # action for red color

     # the end of color code
     #***************************************************************************************************************************
    '''''
    #to save the data in file
    signal.append(u)
    errors.append(error)
    #speed.append(car_speed)
    time_ms.append(t)
    '''



    #print("t :", t)
    print("u :", u)
    print("cicle num :", number_of_reads)
    #if not first:
        #print("delta_t : ", delta_t)
    print("######################")


    cv2.imshow("masked", masked)
    #cv2.imshow("frame", frame)
    #cv2.imshow("edges", edges)
    cv2.imshow("video", orig_frame)
    cv2.imshow("masked0", masked_s)

    first = False
    key = cv2.waitKey(1)
    if key == 27:
        break
"""""        
#stor the data        
with open (file_name,'w') as f :
    f.write (str(signal))
    f.write(str(errors))
    f.write(str(time_ms))
    #f.write(str(car_speed))
"""
video.release()
cv2.destroyAllWindows()