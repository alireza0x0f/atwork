import numpy as np
import cv2
import random as rng
import cmath
import math
FOR_CIRCLE=1

#************************************************************************************************************* find cicle in image
def cicle_dec(frame):
    global show
    global mohretoX
    global mohretoY
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 1)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.3, 20,param1=300,param2=90,minRadius=10,maxRadius=50)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            #cv2.circle(show,(i[0],i[1]),i[2],(0,255,0),2)
            #cv2.circle(show,(i[0],i[1]),2,(0,0,255),3)
            if i[2]>24 and i[2]<37:
                cropped = frame[i[1]-50:i[1]+50,i[0]-50:i[0]+50]
                if cropped.size>0:
                    crop = cv2.convertScaleAbs(cropped, 3, 0.8)
                    out=color_dec(crop,1)
                    if out<10:
                        cv2.putText(show,"RING!!!",(i[0],i[1]), 0, 1, 255)
                    else:

                        gray1 = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
                        gray1 = cv2.medianBlur(gray1, 1)
                        circles = cv2.HoughCircles(gray1, cv2.HOUGH_GRADIENT, 1.5, 200,param1=100,param2=30,minRadius=0,maxRadius=0)
                        if circles is not None:
                            for j in circles[0,:]:
                                if j[2]>i[2]/2-5 and j[2]<i[2]/2+5 and j[2]>10:
                                    crop= line_dec(crop)
                                    mohretoX=i[0]
                                    mohretoY=i[1]
                                    cv2.putText(show,"bearing!!!",(i[0],i[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))


    return gray
#*************************************************************************************************************def cicle_dec(frame):
#************************************************************************************************************* find cicle in image
def cicle_dec_ring(frame):
    global show
    global mohretoX
    global mohretoY
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 1)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 20,param1=300,param2=70,minRadius=24,maxRadius=37)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            #cv2.circle(show,(i[0],i[1]),i[2],(0,255,0),2)
            #cv2.circle(show,(i[0],i[1]),2,(0,0,255),3)
            if i[2]>24 and i[2]<40:
                cropped = frame[i[1]-50:i[1]+50,i[0]-50:i[0]+50]
                if cropped.size>0:
                    crop = cv2.convertScaleAbs(cropped, 3, 1.5)
                    out=color_dec(crop,1)
                    print out
                    if out<10:
                        cv2.putText(show,"ring!!!",(i[0],i[1]), 2, 0.5, (255, 0, 0))

    return gray
#*************************************************************************************************************def cicle_dec(frame):

#************************************************************************************************************* find color for cicle in image
def color_dec(frame,mode):
    maw=0
    blueLower = (0,0,0)  #100,130,50
    blueUpper = (255,255,50) #200,200,130
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.medianBlur(hsv, 3)
    mask = cv2.inRange(hsv, blueLower, blueUpper)
    _,contours, hierarchy = cv2.findContours(mask, 2, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area>150:
            contour_list.append(contour)
        if area > maw :

            maw=area
    if mode==1:
        return maw
    elif mode==2:
        return contour_list
    else:
        return frame
#*************************************************************************************************************
#************************************************************************************************************* find line in image
def line_dec(frame):
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,70,200)
    minLineLength = 0
    maxLineGap = 0
    lines = cv2.HoughLines(edges, 1.3, np.pi / 180, 50, None, 0, 0)
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(frame, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

    return edges
#*************************************************************************************************************
#************************************************************************************************************* find color in image
def Hull_dec(frame,val):
    threshold = val
    src_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    src_gray = cv2.blur(src_gray, (2,2))
    # Detect edges using Canny
    canny_output = cv2.Canny(src_gray, 100, 80)
    # Find contours
    _, contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_list=color_dec(frame,2)
    # Find the convex hull object for each contour
    hull_list = []
    apox_list=[]
    allpoint0=[]
    allpoint1=[]
    maximum=10

    for i in range(len(contours)):
        defects=None
        approx = cv2.approxPolyDP(contours[i],3,True)
        area = cv2.contourArea(approx)
        if area>0 and len(contour_list)>2:
            M = cv2.moments(approx)

            cX = float(M["m10"]/M["m00"])
            cY = float(M["m01"]/M["m00"])
            for q in range(len(contour_list)):
                M = cv2.moments(contour_list[q])
                cv2.drawContours(frame, contour_list, q, (255,255,0))
                cXJ = float(M["m10"]/M["m00"])
                cYJ = float(M["m01"]/M["m00"])
                if abs(cX-cXJ)<150 and abs(cY-cYJ)<150:
                    approx=None




            if approx is not None:
        # hull = cv2.convexHull(approx,returnPoints = False)
                cnt = approx
                hull = cv2.convexHull(cnt,returnPoints = False)
                hull_list.append(hull)
                defects = cv2.convexityDefects(cnt,hull)

        if defects is not None:
            for i in range(defects.shape[0]):
                s,e,f,d = defects[i,0]
                start = tuple(cnt[s][0])
                end = tuple(cnt[e][0])
                far = tuple(cnt[f][0])
                allpoint0.append(far[0])
                allpoint1.append(far[1])

                cv2.line(frame,start,end,[0,255,0],2)
                cv2.circle(frame,far,5,[0,i*2,0],-1)
        dispoint=[]
        for i in range(len(allpoint0)):
            for j in range(len(allpoint0)):
                bb=(allpoint0[i]-allpoint0[j])
                dd=(allpoint1[i]-allpoint1[j])
                qq=(allpoint0[i]+allpoint0[j])/2
                ww=(allpoint1[i]+allpoint1[j])/2
                po=cmath.sqrt(bb*bb+dd*dd)
                if po.real<434 and po.real>430:
                    print "ok"
                if po.real>412 and po.real<417:
                    print "er2"
                if po.real>330 and po.real<334:
                     cv2.putText(show,"mill",(qq,ww), 0, 1, 255)
                     print "mmmmm"


                if po.real>maximum:
                    maximum=po.real
    for i in range(len(contours)):
        color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        #cv2.drawContours(frame, contours, i, color)
        #cv2.drawContours(frame, hull_list, i, color)
    #print maximum
    # Show in a window
    return frame

#*************************************************************************************************************
#*************************************************************************************************************
def find_if_close(cnt1,cnt2):
    row1,row2 = cnt1.shape[0],cnt2.shape[0]
    for i in xrange(row1):
        for j in xrange(row2):
            dist = np.linalg.norm(cnt1[i]-cnt2[j])
            if abs(dist) < 100 :
                return True
            elif i==row1-1 and j==row2-1:
                return False

#**********************************************************************************************************
#*************************************************************************************************************
#************************************************************************************************************* find color for cicle in image
def color_dec_obj(frame,mode):
    global motor
    global pro40
    global pro20
    global pich
    global obj
    global mohre
    global show
    blueLower = (0,0,0)  #100,130,50
    blueUpper = (255,255,70) #200,200,130
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.medianBlur(hsv, 17)
    mask = cv2.inRange(hsv, blueLower, blueUpper)
    _,contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []
    maxq=0
    maxcont=None
    if mode==1:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > maxq :
                approx = cv2.approxPolyDP(contour,3,True)
                contour_list.append(approx)
                maxq=area
                maxcont=approx
        print maxq
        return maxcont
    else:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50 :
                approx = cv2.approxPolyDP(contour,3,True)
                contour_list.append(approx)
                #---------------------MATCHING-----------------------------------------------------------------
                #**********************************************************************************************
                #----------------------------------------------------------------------------------------------
                #$$$$$$$$$$$$$$$$$$$motor$$$$$$$$$$$$$$$$$
                if motor is not None:
                    allpoint0=[]
                    allpoint1=[]
                    maximum=2
                    ret = cv2.matchShapes(approx,motor,1,0.0)
                    if ret<0.4:

                        cnt = contour
                        hull = cv2.convexHull(cnt,returnPoints = False)
                        defects = cv2.convexityDefects(cnt,hull)
                        if defects is not None:
                            for i in range(defects.shape[0]):
                                s,e,f,d = defects[i,0]
                                start = tuple(cnt[s][0])
                                end = tuple(cnt[e][0])
                                far = tuple(cnt[f][0])
                                allpoint0.append(end[0])
                                allpoint1.append(end[1])
                                #cv2.line(frame,start,end,[0,255,0],2)
                                #cv2.circle(frame,far,5,[0,255,0],-1)
                        dispoint=[]
                        for i in range(len(allpoint0)):
                            for j in range(len(allpoint0)):
                                bb=(allpoint0[i]-allpoint0[j])
                                dd=(allpoint1[i]-allpoint1[j])
                                po=cmath.sqrt(bb*bb+dd*dd)
                                if po.real>maximum:
                                    maximum=po.real
                        M = cv2.moments(cnt)
                        cX = int(M["m10"]/M["m00"])
                        cY = int(M["m01"]/M["m00"])
                        if maximum>170 and maximum<180:

                            cv2.putText(show, "motor!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                                        #+++++++++++++end_motor+++++++++++++++++++
                #$$$$$$$$$$$$$$$$$$$pro40$$$$$$$$$$$$$$$$$
                if pro40 is not None:
                    allpoint0=[]
                    allpoint1=[]
                    maximum=2
                    ret = cv2.matchShapes(approx,motor,1,0.0)
                    if ret<0.4:

                        cnt = contour
                        hull = cv2.convexHull(cnt,returnPoints = False)
                        defects = cv2.convexityDefects(cnt,hull)
                        if defects is not None:
                            for i in range(defects.shape[0]):
                                s,e,f,d = defects[i,0]
                                start = tuple(cnt[s][0])
                                end = tuple(cnt[e][0])
                                far = tuple(cnt[f][0])
                                allpoint0.append(end[0])
                                allpoint1.append(end[1])
                                #cv2.line(frame,start,end,[0,255,0],2)
                                #cv2.circle(frame,far,5,[0,255,0],-1)
                        dispoint=[]
                        for i in range(len(allpoint0)):
                            for j in range(len(allpoint0)):
                                bb=(allpoint0[i]-allpoint0[j])
                                dd=(allpoint1[i]-allpoint1[j])
                                po=cmath.sqrt(bb*bb+dd*dd)
                                if po.real>maximum:
                                    maximum=po.real
                        M = cv2.moments(cnt)
                        cX = int(M["m10"]/M["m00"])
                        cY = int(M["m01"]/M["m00"])
                        #print maximum
                        if maximum>220 and maximum<238:

                            cv2.putText(show, "40*40!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        if maximum>175 and maximum<185:

                            cv2.putText(show, "motor!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                                        #+++++++++++++end_pro40+++++++++++++++++++
                #$$$$$$$$$$$$$$$$$$$pro20$$$$$$$$$$$$$$$$$
                if pro20 is not None:
                    allpoint0=[]
                    allpoint1=[]
                    maximum=2
                    ret = cv2.matchShapes(approx,pro20,1,0.0)
                    if ret<0.4:

                        cnt = contour
                        hull = cv2.convexHull(cnt,returnPoints = False)
                        defects = cv2.convexityDefects(cnt,hull)
                        if defects is not None:
                            for i in range(defects.shape[0]):
                                s,e,f,d = defects[i,0]
                                start = tuple(cnt[s][0])
                                end = tuple(cnt[e][0])
                                far = tuple(cnt[f][0])
                                allpoint0.append(end[0])
                                allpoint1.append(end[1])
                                #cv2.line(frame,start,end,[0,255,0],2)
                                #cv2.circle(frame,far,5,[0,255,0],-1)
                        dispoint=[]
                        for i in range(len(allpoint0)):
                            for j in range(len(allpoint0)):
                                bb=(allpoint0[i]-allpoint0[j])
                                dd=(allpoint1[i]-allpoint1[j])
                                po=cmath.sqrt(bb*bb+dd*dd)
                                if po.real>maximum:
                                    maximum=po.real
                        M = cv2.moments(cnt)
                        cX = int(M["m10"]/M["m00"])
                        cY = int(M["m01"]/M["m00"])
                        #print maximum
                        if maximum>190 and maximum<210:

                            cv2.putText(show, "20*20!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                                        #+++++++++++++end_pro20+++++++++++++++++++
                #$$$$$$$$$$$$$$$$$$$pich$$$$$$$$$$$$$$$$$
                if pich is not None:
                    allpoint0=[]
                    allpoint1=[]
                    maximum=2
                    ret = cv2.matchShapes(approx,pich,1,0.0)
                    if ret<0.4:

                        cnt = contour
                        hull = cv2.convexHull(cnt,returnPoints = False)
                        defects = cv2.convexityDefects(cnt,hull)
                        if defects is not None:
                            for i in range(defects.shape[0]):
                                s,e,f,d = defects[i,0]
                                start = tuple(cnt[s][0])
                                end = tuple(cnt[e][0])
                                far = tuple(cnt[f][0])
                                allpoint0.append(end[0])
                                allpoint1.append(end[1])
                                #cv2.line(frame,start,end,[0,255,0],2)
                                #cv2.circle(frame,far,5,[0,255,0],-1)
                        dispoint=[]
                        for i in range(len(allpoint0)):
                            for j in range(len(allpoint0)):
                                bb=(allpoint0[i]-allpoint0[j])
                                dd=(allpoint1[i]-allpoint1[j])
                                po=cmath.sqrt(bb*bb+dd*dd)
                                if po.real>maximum:
                                    maximum=po.real
                        M = cv2.moments(cnt)
                        cX = int(M["m10"]/M["m00"])
                        cY = int(M["m01"]/M["m00"])
                        #print maximum
                        if maximum>219 and maximum<230:

                            cv2.putText(show, "pich!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                                        #+++++++++++++end_pich+++++++++++++++++++
               #$$$$$$$$$$$$$$$$$$$mohre$$$$$$$$$$$$$$$$$
                if mohre is not None:
                    global mohretoX
                    global mohretoY
                    allpoint0=[]
                    allpoint1=[]
                    maximum=2
                    ret = cv2.matchShapes(approx,mohre,1,0.0)
                    if ret<0.4:

                        cnt = contour
                        hull = cv2.convexHull(cnt,returnPoints = False)
                        defects = cv2.convexityDefects(cnt,hull)
                        if defects is not None:
                            for i in range(defects.shape[0]):
                                s,e,f,d = defects[i,0]
                                start = tuple(cnt[s][0])
                                end = tuple(cnt[e][0])
                                far = tuple(cnt[f][0])
                                allpoint0.append(end[0])
                                allpoint1.append(end[1])
                                cv2.line(frame,start,end,[0,255,0],2)
                                cv2.circle(frame,far,5,[0,255,0],-1)
                        dispoint=[]
                        for i in range(len(allpoint0)):
                            for j in range(len(allpoint0)):
                                bb=(allpoint0[i]-allpoint0[j])
                                dd=(allpoint1[i]-allpoint1[j])
                                po=cmath.sqrt(bb*bb+dd*dd)
                                if po.real>maximum:
                                    maximum=po.real
                        M = cv2.moments(cnt)
                        cX = int(M["m10"]/M["m00"])
                        cY = int(M["m01"]/M["m00"])
                        #print maximum
                        if maximum>94 and maximum<111:
                            rt=cv2.pointPolygonTest(contour, (cX, cY), True)
                            if rt>38 and rt<44:
                                cv2.putText(show, "large-mohre!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        if maximum>59 and maximum<70 and abs(cX-mohretoX)>20 and abs(cY-mohretoY)>20:
                            cv2.putText(show, "small-mohre!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                                        #+++++++++++++end_pich+++++++++++++++++++
                 #$$$$$$$$$$$$$$$$$$$pich$$$$$$$$$$$$$$$$$
                if obj is not None:
                    allpoint0=[]
                    allpoint1=[]
                    maximum=2
                    ret = cv2.matchShapes(approx,obj,1,0.0)
                    if ret<0.4:

                        cnt = contour
                        hull = cv2.convexHull(cnt,returnPoints = False)
                        defects = cv2.convexityDefects(cnt,hull)
                        if defects is not None:
                            for i in range(defects.shape[0]):
                                s,e,f,d = defects[i,0]
                                start = tuple(cnt[s][0])
                                end = tuple(cnt[e][0])
                                far = tuple(cnt[f][0])
                                allpoint0.append(end[0])
                                allpoint1.append(end[1])
                                #cv2.line(frame,start,end,[0,255,0],2)
                                #cv2.circle(frame,far,5,[0,255,0],-1)
                        dispoint=[]
                        for i in range(len(allpoint0)):
                            for j in range(len(allpoint0)):
                                bb=(allpoint0[i]-allpoint0[j])
                                dd=(allpoint1[i]-allpoint1[j])
                                po=cmath.sqrt(bb*bb+dd*dd)
                                if po.real>maximum:
                                    maximum=po.real
                        M = cv2.moments(cnt)
                        cX = int(M["m10"]/M["m00"])
                        cY = int(M["m01"]/M["m00"])
                        #print area
                        if maximum>94 and maximum<111 :
                            rt=cv2.pointPolygonTest(contour, (cX, cY), True)
                            if rt>25 and rt<30:
                                cv2.putText(show, "obj!!!", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                                        #+++++++++++++end_pich+++++++++++++++++++

                #eeeeeeeeeeeeeeeeeeeeeeeeennnnnnnnnnnnnnnnnnnnnnnnnnnnnnndddddddddddddd(((((match)))))
        cv2.drawContours(frame, contour_list,  -1, (255,0,0), 2)

                #eeeeeeeeeeeeeeeeeeeeeeeeennnnnnnnnnnnnnnnnnnnnnnnnnnnnnndddddddddddddd(((((match)))))
        cv2.drawContours(frame, contour_list,  -1, (255,0,0), 2)

        return frame
def findco(frame):
    threshold =100
    img=frame
    src_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    src_gray = cv2.blur(src_gray, (3,3))
    # Detect edges using Canny
    canny_output = cv2.Canny(src_gray, threshold, 2*threshold /3)
    # Find contours
    _, contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    LENGTH = len(contours)
    status = np.zeros((LENGTH,1))

    for i,cnt1 in enumerate(contours):
        x = i
        if i != LENGTH-1:
            for j,cnt2 in enumerate(contours[i+1:]):
                x = x+1
                dist = find_if_close(cnt1,cnt2)
                if dist == True:
                    val = min(status[i],status[x])
                    status[x] = status[i] = val
                else:
                    if status[x]==status[i]:
                        status[x] = i+1
    return frame
'''
    unified = []
    maximum = int(status.max())+1
    for i in xrange(maximum):
        pos = np.where(status==i)[0]
        if pos.size != 0:
            cont = np.vstack(contours[i] for i in pos)
            hull = cv2.convexHull(cont)
            unified.append(hull)

   # cv2.drawContours(img,unified,-1,(0,255,0),2)
    #cv2.drawContours(img,unified,-1,255,-1)
    '''
def cos_dec():
    global motor
    global pro40
    global pro20
    global pich
    global obj
    global mohre
    frame = cv2.imread('motor.png')

    if frame is not None:
        motor=color_dec_obj(frame,1)

    else:
        motor=None
        print "img not found************************************************************"
    #1111111111111111111111111111111111111111111
    frame = cv2.imread('pro40.png')
    if frame is not None:
        pro40=color_dec_obj(frame,1)
    else:
        pro40=None
        print "img not found*************************************************************"
    #1111111111111111111111111111111111111111111
    frame = cv2.imread('pro20.png')
    if frame is not None:
        pro20=color_dec_obj(frame,1)
    else:
        pro20=None
        print "img not found*************************************************************"
    #1111111111111111111111111111111111111111111
    frame = cv2.imread('pich.png')
    if frame is not None:
        pich=color_dec_obj(frame,1)
    else:
        pich=None
        print "img not found*************************************************************"
    #1111111111111111111111111111111111111111111
    frame = cv2.imread('mohre.png')
    if frame is not None:
        mohre=color_dec_obj(frame,1)
    else:
        mohre=None
        print "img not found*************************************************************"
    #1111111111111111111111111111111111111111111
    frame = cv2.imread('obj.png')
    if frame is not None:
        obj=color_dec_obj(frame,1)
    else:
        obj=None
        print "img not found*************************************************************"


cap = cv2.VideoCapture(1)
if __name__ == '__main__':
    global show
    cos_dec()
    global mohretoX
    global mohretoY
    mohretoX=0
    mohretoY=0
    while(cap.isOpened()):
        rec,frame=cap.read()
        show=frame
        crop = cv2.convertScaleAbs(frame, 3, 1.5)
        #color_dec_obj(crop,0)
        #cicle_dec(frame)
        #cicle_dec_ring(frame)
        show=Hull_dec(frame,50)
        cv2.imshow('frame',show)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
