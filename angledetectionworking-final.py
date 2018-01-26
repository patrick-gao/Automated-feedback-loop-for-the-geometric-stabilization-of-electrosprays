import cv2 as cv
import math
import numpy as np
import imutils as imt
import serial
import time
from PID_loop import PID
import os
output_V = 1.0/6.0*3.3
p=PID(0.0008,0.00,.2) #PID values (P,I,D)
#MPC4725

def VideoStream(input): #initialize video capture
    cap = cv.VideoCapture(input)
    cap.set(3,1280) #Resolution x
    cap.set(4,720) #Resolution y
    cap.set(5,60) #Framerate
    return cap

def readline(): #read line function from Arduino
    eol = b';' #end of line character
    leneol = len(eol)
    line = bytearray()
    while True:
        c = ser.read(1)
        if c:
            line += c
            if line[-leneol:] == eol:
                break
        else:
            break
    return bytes(line)

def videoprocessing(x1,ra1,style,setpoint,file,pic_interval):
    cap = VideoStream(0)
    ret, frame = cap.read()
    frame = frame[200:600,300:1000]
    gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY) #convert to grayscale
    gray = cv.GaussianBlur(gray,(5,5),0) #apply Gaussian blur
    thresh = cv.threshold(gray,100,255,cv.THRESH_BINARY)[1] #apply threshold
    thresh = abs(255-thresh) #invert colors

    #find contours
    cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imt.is_cv2() else cnts[1]
    c = max(cnts,key=cv.contourArea)

    #left most point
    extLeft = tuple(c[c[:,:,0].argmin()][0])


    #p=PID(0.0008,0.00,.8)
    p.setPoint(setpoint)
    
    

    def imageprocessing(ra,ra1):
        ret, frame = cap.read()
        frame = frame[200:600,400:1100] #window size [y:x]
        gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray,(5,5),0)
        thresh = cv.threshold(gray,100,255,cv.THRESH_BINARY)[1]
        thresh = abs(255-thresh)

        #find contours
        cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imt.is_cv2() else cnts[1]
        c = max(cnts,key=cv.contourArea)

        #left most point
        extLeft = tuple(c[c[:,:,0].argmin()][0])

        def create_blank(width, height, rgb_color=(0, 0, 0)): #creates blank image
            image = np.zeros((height, width, 3), np.uint8)
            color = tuple(reversed(rgb_color))
            image[:] = color
            return image
        
        def findMidpoint(x): #finds midpoint between between 2 intersections for drawing horizon line
                width, height = frame.shape[1], frame.shape[0]
                line = create_blank(width, height, rgb_color=(0,0,0))
                contour = create_blank(width, height, rgb_color=(0,0,0))

                cv.drawContours(contour,[c],-1,(255,255,255),1)
                cv.line(line,(x,0),(x,line.shape[1]),(255,255,255),1)
                andimg = cv.bitwise_and(line,contour)
                intersection = np.transpose(np.nonzero(andimg))
                try: 
                    p1 = (tuple(intersection[0]))
                    p2 = (tuple(intersection[3]))
                except IndexError:
                    p1,p2 = (0,0),(0,0)
                midpoint = (p1[1],((p1[0]+p2[0])/2))
                return midpoint

        def drawLine(p1,p2): #extrapolates horizon line towards the left
            #slope = (p1[1]-p2[1])/(p1[0]-p2[0])
            delta_x = (p1[0]-p2[0])
            delta_y = (p1[1]-p2[1])
            left_point = (0,p1[1]+5*delta_y)
            #int(p1[1]-float(p1[0]/delta_x)*delta_y
            right_point = (500,p1[1])
            #print left_point,right_point
            #int(p1[1]+((float(frame.shape[0]-p1[0]))/delta_x)*delta_y)
            cv.line(frame,left_point,right_point,(255,255,255),1)

        def drawHorizon(): #draws horizon line
            h1 = findMidpoint(500)
            h2 = findMidpoint(600)
            drawLine(h1,h2)
            cv.line(frame,h1,h2,(255,255,255),1)
            
        def findAngle(img1,img2,rgbcolor,ra,ra1): #calculates angle
            andimg = cv.bitwise_and(img1,img2) #performs and operation on two images
            intersections = np.transpose(np.nonzero(andimg)) #creates array of nonzero points in andimg

            if len(intersections) == 2:
                anglefound = 1
                p1 = (tuple(intersections[0]))
                p2 = (tuple(intersections[1]))
                v1 = [p1[1]-extLeft[0],p1[0]-extLeft[1]]
                v2 = [p2[1]-extLeft[0],p2[0]-extLeft[1]]
                a = cv.line(frame,extLeft,(p1[1],p1[0]),rgbcolor)
                b = cv.line(frame,extLeft,(p2[1],p2[0]),rgbcolor)
                angle = math.acos(np.dot(v1,v2)/(np.sqrt(np.dot(v1,v1))*np.sqrt(np.dot(v2,v2))))*(180/math.pi) #dot product
                if len(str(angle))>3:
                    return angle,anglefound
                else:
                    anglefound = 0 #no angle found
                    return 180.0,anglefound

            else:
                anglefound = 0
                return 180.0,anglefound

        def drawImages(type,radius,color): #draws bw images for calculating intersections
            width, height = frame.shape[1], frame.shape[0]
            blank = create_blank(width, height, rgb_color=(0,0,0))

            if type == 1: #contour
                cv.drawContours(blank,[c],-1,(255,255,255),1)
                cv.drawContours(frame,[c],-1,color,1)
            elif type == 0: #line
                cv.line(blank,(radius,1),(radius,blank.shape[1]),(255,255,255),1)
                cv.line(frame,(radius,1),(radius,blank.shape[1]),(255,255,255),1)

                #cv.circle(blank,extLeft,radius,(255,255,255),1)
                #cv.circle(frame,extLeft,radius,color,1)

            return cv.cvtColor(blank,cv.COLOR_BGR2GRAY)
            
        def angledetection(extLeft,c,ra,ra1): #calculating angles
            contourGray = drawImages(1,0,(255,255,255))
            ocircleGray = drawImages(0,trackbar,(0,255,0))
            icircleGray = drawImages(0,extLeft[0]+ra1,(255,0,0))

            #find two angles
            angle1,l_anglefound = findAngle(contourGray,ocircleGray,(0,255,0),ra,ra1)
            angle2,s_anglefound = findAngle(contourGray,icircleGray,(255,0,0),ra,ra1)

            return (angle1,angle2),(l_anglefound,s_anglefound)
        
        angles,angle_found = angledetection(extLeft,c,ra,ra1)
        drawHorizon()

        if angle_found[0] == 0:
            pid = -5
            cv.imshow('Cone Angle Detection',frame)
        else:
            pid = p.update(angles[0]) #updates PID loop
            #puts text on frame
            cv.putText(frame,'Cone Angle = %.3f'%angles[0],(ra1+10,(frame.shape[1]/20)*3),cv.FONT_HERSHEY_SIMPLEX,.5,(0,255,0))
            cv.putText(frame,'Tip Angle = %.3f'%angles[1],(ra1+10,(frame.shape[1]/20)*4),cv.FONT_HERSHEY_SIMPLEX,.5,(255,0,0))
            cv.putText(frame,'Outer Radius = %.0f'%ra,(ra1+10,(frame.shape[1]/20)),cv.FONT_HERSHEY_SIMPLEX,.5,(0,0,0))
            cv.putText(frame,'Inner Radius = %.0f'%ra1,(ra1+10,(frame.shape[1]/20)*2),cv.FONT_HERSHEY_SIMPLEX,.5,(0,0,0))
  
        def voltagetest(voltagetesting,output_V,angles):
            if voltagetesting == 1: #for testing range of voltage
                output_V = output_V +.01
                if output_V > (4.9):
                    output_V = 0.1
            if voltagetesting == 0: #working PID loop, controls limits of outputV
                output_V = output_V + pid/10.0
                if output_V > 4.9:
                        output_V = 4.9
                elif output_V < 0.1:
                        output_V =0.1
            return output_V

        global output_V
        output_V = voltagetest(0,output_V,angles)
        ser.write('%.6f;'%output_V) #writes output V to Arduino
        tipangle = '%.3f'%angles[1]
        baseangle = '%.3f'%angles[0]
        outputV = '%.3f'%output_V
        pidstr = '%.3f'%pid
        arduino = readline() #reads lines from Arduino
        cv.imshow('Cone Angle Detection',frame)
        print ' {} , {} , {} , {} , {} , {} , {} , {}'.format(tipangle,baseangle,arduino[:-1],outputV,pidstr,PID.getPoint(p),trackbar,p.Kp)
        file.write(' {} , {} , {} , {} , {} , {} , {}\n'.format(tipangle,baseangle,arduino[:-1],outputV,pidstr,PID.getPoint(p),trackbar))
        return frame,extLeft,angles

    cv.namedWindow('Cone Angle Detection')
    cv.startWindowThread()

    pic_count = 0
    #creating folder names
    datename = 'Screenshots/{}'.format(time.strftime('%Y-%m-%d'))
    hourname = 'Screenshots/{}/{}'.format(time.strftime('%Y-%m-%d'),time.strftime('%H-%M-%S'))

    if pic_interval > 0:
        i = 0
    try:
        os.mkdir(datename)
    except WindowsError:
        pass
    os.mkdir(hourname)

    while(True):
        trackbar = cv.getTrackbarPos('Cone Base','Control') #reads value of trackbar for cone base
        ra = trackbar-extLeft[0] #calculates length from cone base line to extLeft
        frame,extLeft,angles = imageprocessing(ra,ra1)
        pic_count += 1
        if pic_interval > 0:
            if pic_count >= pic_interval:
                cv.imwrite('{}/Frame{}.jpg'.format(hourname,i),frame) #writes screenshots
                pic_count = 0
                i+=1
        if cv.waitKey(int(style)) & 0xFF == ord('q'): #press q to quit
            cv.destroyWindow('Cone Angle Detection')
            break
        if cv.waitKey(int(style)) & 0xFF == ord('f'): #press f to invert Kp
            p.setKp(p.Kp*(-1))
            
    
    cap.release()
    return datename,hourname

def nothing(x):
    pass

def getTrackbarPos(x):
    return PID.setPoint(p,cv.getTrackbarPos('Set Point','Control'))

def main():
    file = open('angledata.txt','w')
    x1 = int(raw_input('Input cone base: '))
    file.write('Cone base = %.0f\n'%x1)
    ra1 = int(raw_input('Input inner radius: '))
    file.write('Tip angle length = %.0f\n'%ra1)
    style = raw_input('Continuous[1] or intermittent[0]? ')
    setpoint = float(raw_input('Set point: '))
    file.write('Set Point = %.1f\n'%setpoint)
    pic_interval = int(raw_input('Screenshot interval?: '))
    file.write('Screenshot interval = %.0f\n'%pic_interval)
    print ("Mason 112 Pressure and voltage sensor\n### V1.1 ###\nData update rate = 1 Hz\nData adquisition rate = 100 Hz\nUnits Pressure = kPa\nUnits Voltage = mV\n Tip Angle (degrees), Cone Angle (degrees), Output V (V), P (kPa), Ch1 (mV), Ch2 (mV), 12 bit output, Measured Out V (mV), PID\n")
    file.write("Mason 112 Pressure and voltage sensor\n### V1.1 ###\nData update rate = 1 Hz\nData adquisition rate = 100 Hz\nUnits Pressure = kPa\nUnits Voltage = mV\n Tip Angle (degrees), Cone Angle (degrees), Output V (V), P (kPa), Ch1 (mV), Ch2 (mV), 12 bit output, Measured Out V (mV), PID\n")
    cv.namedWindow('Control')
    cv.createTrackbar('Cone Base','Control',x1,700,nothing) #creates trackbar for cone base
    cv.createTrackbar('Set Point','Control',int(setpoint),180,getTrackbarPos) #creates trackbar for set point
    datename,hourname = videoprocessing(x1,ra1,style,setpoint,file,pic_interval)

    file_save = raw_input('Save data? (y/n): ')
    if file_save == 'y':
        file.close()
        try:
            #os.rename('angledata.txt','C:/Users/Marcelo/Documents/MATLAB/AFOSR/PIDcontrol.txt')
            os.rename('angledata.txt','{}/{}.txt'.format(hourname,raw_input('File name: ')))
        except WindowsError:
            print 'Screenshots/{}/{}/{}.txt'.format(datename,hourname,raw_input('File name: '))
    else:
        file.close()
        os.remove('angledata.txt')
    x = raw_input('\nAgain? (y/n): ')
    if x == 'y':
        print'\n'
        main()
    elif x == 'n':
        cv.destroyAllWindows()

ser = serial.Serial('COM6',115200,timeout=0) #initializes serial connection
main()