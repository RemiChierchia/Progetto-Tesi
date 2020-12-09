import numpy as np
import argparse
import cv2 as cv
import time
import math
import struct
import json

#global variabiles
left_direction = False #defined global so i can access everywhere
right_direction = False
frames_passed = 0
time_right = 0
time_right_previous = 0
time_left = 0
time_left_previous = 0
ball_counter = 0
gravity = 9.8
previous_x = 0
previous_y = 0
candidate_x = 0
candidate_y = 0
not_ball = True
velocity_xr = 0
velocity_yr = 0
velocity_xl = 0
velocity_yl = 0
ball_mass = 0.058 #56-59,4g
time_contact = 0.005 * 40 # * la dilatazione temporale all'incirca
values_r = []
values_l = []
fps = 0
player_height = 615 #pixels has been mesured the height
player_mass = 70 #kg estimated
last_right_velocity = 0
force_ready = False
I_rigidbody = 1/3
l_impact_wrist = 125 #pixels 45cm
player_mass = 70
player_hand = player_mass/100 * 0.612
player_arm = player_mass/100 * 2.64
player_forearm = player_mass/100 * 1.531
player_height = 615 #pixels mesured
l_forearm = player_height/100 * 14.5 #89 pixel
l_arm = player_height/100 * 18.8
raquet_mass = 0.3
gravity = 9.81
degrees_2_radiants = (math.pi*2)/360
clock_wise_momentum_sholder = False
non_clock_wise_momentum_sholder = False
angle_wrist_raquet = 100
#

def background_substractor(img, background_img, kernel = np.ones((3,3),np.uint8)):

    diffRGB = cv.absdiff(img, background_img)
    _, frame_BIN = cv.threshold(diffRGB, 10, 255, cv.THRESH_BINARY) #white is the moving stuff #best settings for lateral video
    frame_grey = cv.cvtColor(frame_BIN, cv.COLOR_BGR2GRAY)
    #cv.imshow('frame bin', frame_grey)
    frame_grey = cv.morphologyEx(frame_grey, cv.MORPH_OPEN, kernel, iterations = 12) #best settings for lateral video
    #cv.imshow("test", frame_grey)
    #frame_grey = cv.morphologyEx(frame_grey, cv.MORPH_CLOSE, kernel, iterations = 4)

    return frame_grey

def player_tracking(player_mask, mode, method): #with cloud around the player

    contours, _ = cv.findContours(player_mask, mode, method)
    #Find the convex hull object for each contour or all
    concatenated = np.vstack(contours) #concatenates all contours, given that none of them is of the ball
    hull_list = []
    #for i in range(len(contours)):
    hull = cv.convexHull(concatenated) #saves the external points
    hull_list.append(hull) #connects the external points
    #Draw contours + hull results
    player_hull = np.zeros((player_mask.shape[0], player_mask.shape[1], 3), dtype=np.uint8)
    #for i in range(len(contours)):
    cv.drawContours(player_hull, hull_list, -1, color = (255,255,255)) #draws the external contour
    #cv.fillPoly(player_hull, hull_list, color = (255, 255, 255)) #fills the contour
    cv.drawContours(player_hull, concatenated, -1, color = (255,255,255)) #draws only the player contours
    #cv.rectangle(player_hull, hull_list, color = (255, 255, 255))
    #cv.imshow('Contours_player', player_hull)

    left_most_points = tuple(concatenated[concatenated[:, :, 0].argmin()][0]) #return minimun x value of the player contour
    top = tuple(concatenated[concatenated[:, :, 1].argmin()][0])
    bottom = tuple(concatenated[concatenated[:, :, 1].argmax()][0])
    print(top, bottom)
    #print("position min of the player =", left_most)
    #if left_most_points != 0:
    left_most = left_most_points[0]

    return player_hull, left_most

def ball_tracking(ball_mask, result, mode, method):

    #global possible_ball
    global candidate_x
    global candidate_y

    ball_mask = cv.morphologyEx(ball_mask, cv.MORPH_OPEN, np.ones((3,3),np.uint8), iterations = 1)
    ball_mask = cv.morphologyEx(ball_mask, cv.MORPH_CLOSE, np.ones((3,3),np.uint8), iterations = 1)
    #cv.imshow("ball_mask", ball_mask)

    contours, _ = cv.findContours(ball_mask, mode, method)
    concatenated = np.vstack(contours) #concatenates all contours, given that none of them is of the ball
    #cv.drawContours(ball_mask, contours, -1, (0,255,0), 3) #draw all contours
    #arclength = cv.arcLength(concatenated, True)

    candidate_x = 0
    candidate_y = 0
    c_xy = 0
    c_yy = 0
    time = []
    # calculate area and circular shape
    for c in contours: #settings for video rallenty lateral, because the oblique has a little much of shadows
        area = cv.contourArea(c)
        if 200 < area < 500: #simil ball area due to low frame rate; nearest is the ball bigger it becomes
        #circularity check not effective due to 25fps in fact the ball isnt circular
        #but with the video rallenty it is perfect
            perimeter = cv.arcLength(c, True)
            area = cv.contourArea(c)
            circularity= 4*math.pi*(area/(perimeter*perimeter));

            if 0.7 < circularity < 1.2:
                #print(circularity) #in rallenty mode the circularity is around 0.7-0.9
                ellipse = cv.fitEllipse(c)
                result = cv.ellipse(frame, ellipse, (0,255,0), 2)
                M = cv.moments(c)
                c_x = int(M["m10"] / (M["m00"] + 1e-5)) #+1e-5 to avoid division by 0
                c_y = int(M["m01"] / (M["m00"] + 1e-5))
                """
                for y in yellow_contours: #if no yellow stuff is detected, no stuff is detected anyway
                    if cv.contourArea(y) > 5:
                        M_y = cv.moments(y)
                        c_xy = int(M_y["m10"] / (M_y["m00"] + 1e-5)) #+1e-5 to avoid division by 0
                        c_yy = int(M_y["m01"] / (M_y["m00"] + 1e-5))
                        print(c_xy,c_yy)
                if c_xy-2<= c_x <=c_xy+2 and c_yy-2<= c_y <=c_yy+2: #spprox between detections
                """
                candidate_x = c_x
                candidate_y = c_y
                print(candidate_x, candidate_y)
                #ball_counter += 1
                #cv.circle(result, (int(c_x), int(c_y)), 4, (0,255,0), -1)
    #left_most = tuple(concatenated[concatenated[:, :, 0].argmin()][0])
    cv.imshow("ball_contours", result)
    #print("ball maybe", left_most)

    return result

def direction(precedent_x, left_most, start,):

    global candidate_x #use global to change global variable, otherwise are defined local
    global candidate_y
    global left_direction
    global right_direction
    global frames_passed
    global ball_counter
    global not_ball
    global last_right_velocity
    global velocity_xr

    print("start", candidate_x)

    if candidate_x == 0:
        if frames_passed-ball_counter >= 3:
            if right_direction == True:
                print("start hit", frames_passed)
            left_direction = False
            right_direction = False
    else:
        if candidate_x < left_most-15: #approximation #not so good #in rallenty mode the circularity of the ball is quite perfect so i can remove approximation
            not_ball = False
            print("player", left_most, "pallaaa", candidate_x) #enters only if there is a valid ball
            ball_counter += 1
            if frames_passed-ball_counter < 3:
                print("intresting movies", frames_passed)
                if candidate_x < precedent_x: #it needs at least two values
                    left_direction = True
                    precedent_x = candidate_x
                    print("leeeeftt")
                elif candidate_x > precedent_x:
                    right_direction = True
                    precedent_x = candidate_x
                    print("rigggghtt")
                else:
                    precedent_x = candidate_x
            else:
                ball_counter = frames_passed
                left_direction = False #not needed
                right_direction = False
        else:
            if right_direction == True:
                last_right_velocity = traject_velocity_right
                print('last_right_velocity', last_right_velocity)
            not_ball = True

    print("stop")

    return precedent_x

def velocity(fps, traject_velocity_right, traject_velocity_left): #better if do an average every 3-5 values

    global time_right
    global time_right_previous
    global time_left
    global time_left_previous
    global previous_x
    global previous_y
    global velocity_xr
    global velocity_yr
    global velocity_xl
    global velocity_yl
    global frames_passed
    global values_r
    global values_l
    global force_ready

    if right_direction == True: #there could be x and y candidates that are void because i have to wait at least 3 frames to determine change direction or ball lost
        velocity_xl = 0
        velocity_yl = 0
        time_right += 1
        time_left = 0
        time_left_previous = 0
        values_l.clear()
        print("right again", time_right)
        if not_ball == False:
            if previous_x != 0 and previous_y != 0 and time_right >= 2:
                time = (time_right-time_right_previous)/fps
                if time_right == 2:
                    velocity_xr = math.fabs(candidate_x-previous_x)/time
                    velocity_yr = math.fabs(candidate_y-previous_y)/time
                else:
                    values_r.append((math.fabs(candidate_x-previous_x)/time, math.fabs(candidate_y-previous_y)/time))
                    #times.append(time)
                    if len(values_r) > 4: #to fix
                        for i in range (len(values_r)):
                            velocity_xr += values_r[i][0]
                            velocity_yr += values_r[i][1]
                        velocity_xr = velocity_xr/5
                        velocity_yr = velocity_yr/5
                        traject_velocity_right = math.sqrt((velocity_xr)**2 + (velocity_yr)**2)
                        #print("mlmlmlm", values_r[i])
                        values_r.clear()
                        #print(values_r)
                    """
                    velocity_xr = (velocity_xr + (math.fabs(candidate_x-previous_x)/time))/2 #at the first loop previous is 0
                    velocity_yr = (velocity_yr + (math.fabs(candidate_y-previous_y)/time))/2
                    traject_velocity_right = math.sqrt(velocity_xr**2 + velocity_yr**2)
                    print(traject_velocity_right)
                    """
            previous_x = candidate_x
            previous_y = candidate_y
            time_right_previous = time_right
    elif left_direction == True:
        velocity_xr = 0
        velocity_yr = 0
        time_left += 1
        time_right = 0
        time_right_previous = 0
        values_r.clear()
        print("left again")
        if not_ball == False:
            if previous_x != 0 and previous_y != 0 and time_left >= 2:
                time = (time_left-time_left_previous)/fps
                if time_left == 2:
                    print("finish hit", frames_passed)
                    velocity_xl = math.fabs(candidate_x-previous_x)/time
                    velocity_yl = math.fabs(candidate_y-previous_y)/time
                    print('vefeeee', math.sqrt((velocity_xl)**2 + (velocity_yl)**2))
                else:
                    values_l.append((math.fabs(candidate_x-previous_x)/time, math.fabs(candidate_y-previous_y)/time))
                    #times.append(time)
                    if len(values_l) > 4: #to fix
                        for i in range (len(values_l)):
                            velocity_xl += values_l[i][0]
                            velocity_yl += values_l[i][1]
                        velocity_xl = velocity_xl/5
                        velocity_yl = velocity_yl/5
                        traject_velocity_left = math.sqrt((velocity_xl)**2 + (velocity_yl)**2)
                        #print("mlmlmlm", values_l[i])
                        values_l.clear()
                        #print(values_l)
                        force_ready = True
                    print('vefeeee', math.sqrt((velocity_xl)**2 + (velocity_yl)**2))
                    """
                    velocity_xl = (velocity_xl + (math.fabs(candidate_x-previous_x)/time))/2 #at the first loop previous is 0
                    velocity_yl = (velocity_yl + (math.fabs(candidate_y-previous_y)/time))/2
                    traject_velocity_left = math.sqrt(velocity_xl**2 + velocity_yl**2)
                    print(traject_velocity_left)
                    """
            previous_x = candidate_x
            previous_y = candidate_y
            time_left_previous = time_left
    else:
        print("idle")
        time_right = 0
        time_right_previous = 0
        time_left = 0
        time_left_previous = 0
        time = 0
        velocity_xr = 0
        velocity_yr = 0
        velocity_xl = 0
        velocity_yl = 0
        values_r.clear()
        values_l.clear()

    return traject_velocity_right, traject_velocity_left

def force(traject_velocity_right, traject_velocity_left):

    global last_right_velocity
    global force_ready

    elastic_contribution = last_right_velocity*(0.6+0.75) #estimated raquet+ball
    if force_ready == True:
        force = (math.fabs(traject_velocity_left-elastic_contribution) * ball_mass) / time_contact #meno il contributo di racchetta e pallina 0.60 e 0.75
        force_ready = False
        print("force", force)

def json_values():

    global fps
    global player_height

    fps = 25

    I_rigidbody = 1/3
    l_impact_wrist = 125 #pixels 45cm
    player_mass = 70
    player_hand = player_mass/100 * 0.612
    player_arm = player_mass/100 * 2.64
    player_forearm = player_mass/100 * 1.531
    player_height = 615 #pixels mesured
    l_forearm = player_height/100 * 14.5 #89 pixel
    l_arm = player_height/100 * 18.8
    raquet_mass = 0.3
    gravity = 9.81
    degrees_2_radiants = (math.pi*2)/360
    clock_wise_momentum_sholder = False
    non_clock_wise_momentum_sholder = False
    angle_wrist_raquet = 100

    before_Rsholder, before_Relbow, before_Rwrist = [],[],[]

    after_Rsholder, after_Relbow, after_Rwrist = [],[],[]

    load_2_hit = []


    with open('player_move.json') as json_file:
        data = json.load(json_file)
        for index in data['pose']:
            #print(index)
            if int(index) > 123 and int(index) < 327: #range max loading to impact #without int() bug
                col = []
                for i,js in enumerate(data['pose'][index]):
                    if i >= 12 and i <=14:
                        col.append(js)

                load_2_hit.append(col)

            #BEFORE
            if int(index) > 310 and int(index) < 327: #just before impact
                cols, cole, colw = [],[],[]
                for i,js in enumerate(data['pose'][index]):
                    if i >= 6 and i <=8:
                        cols.append(js)
                    elif i >= 9 and i <=11:
                        cole.append(js)
                    elif i >= 12 and i <=14:
                        colw.append(js)

                #print("appp ",len(Rsholder))
                before_Rsholder.append(cols)
                before_Relbow.append(cole)
                before_Rwrist.append(colw)

            #AFTER
            if int(index) > 326 and int(index) < 358: #just before impact
                cols, cole, colw = [],[],[]
                for i,js in enumerate(data['pose'][index]):
                    if i >= 6 and i <=8:
                        cols.append(js)
                    elif i >= 9 and i <=11:
                        cole.append(js)
                    elif i >= 12 and i <=14:
                        colw.append(js)

                #print("appp ",len(Rsholder))
                after_Rsholder.append(cols)
                after_Relbow.append(cole)
                after_Rwrist.append(colw)

    len_before = len(before_Rsholder)-1
    #print(len_before)
    time_before = (len_before+1)/fps


    #print(time_before)
    print("prima 15frames")

    before_shiftx_Rsholder = before_Rsholder[0][0] - before_Rsholder[len_before][0]
    before_vx_Rsholder = before_shiftx_Rsholder/time_before
    before_shifty_Rsholder = before_Rsholder[0][1] - before_Rsholder[len_before][1]
    before_vy_Rsholder = before_shifty_Rsholder/time_before
    before_shift_Rsholder = math.sqrt(before_shiftx_Rsholder**2+before_shifty_Rsholder**2)
    before_v_Rsholder = before_shift_Rsholder/time_before
    before_alpha_Rsholder = math.degrees(math.acos(before_vx_Rsholder/before_v_Rsholder))
    before_w_Rsholder = before_alpha_Rsholder/time_before
    print("Rsholder x,y,tot,v\n", before_shiftx_Rsholder, before_vx_Rsholder, before_shifty_Rsholder, before_vy_Rsholder, before_shift_Rsholder, before_v_Rsholder, before_alpha_Rsholder, before_w_Rsholder)

    #devo verificare al momento dell'impatto com e posizionata la retta spalla gomito

    before_shiftx_Relbow = before_Relbow[0][0] - before_Relbow[len_before][0]
    before_vx_Relbow = before_shiftx_Relbow/time_before
    before_shifty_Relbow = before_Relbow[0][1] - before_Relbow[len_before][1]
    before_vy_Relbow = before_shifty_Relbow/time_before
    before_shift_Relbow = math.sqrt(before_shiftx_Relbow**2+before_shifty_Relbow**2)
    before_v_Relbow = before_shift_Relbow/time_before
    before_alpha_Relbow = math.degrees(math.acos(before_vx_Relbow/before_v_Relbow))
    before_w_Relbow = before_alpha_Relbow/time_before
    print("Relbow x,y,tot,v\n", before_shiftx_Relbow, before_shifty_Relbow, before_shift_Relbow, before_v_Relbow, before_w_Relbow)

    x_sholder_elbow = before_Rsholder[len_before][0]-before_Relbow[len_before][0] #inteoria momento dell'impatto
    y_sholder_elbow = before_Rsholder[len_before][1]-before_Relbow[len_before][1]
    arm_pixel = math.sqrt(x_sholder_elbow**2+y_sholder_elbow**2)
    if y_sholder_elbow < 0:
        y_sholder_elbow = math.fabs(y_sholder_elbow)
    angle_sholder_elbow = math.degrees(math.acos(y_sholder_elbow/arm_pixel))
    #print(angle_sholder_elbow)
    if x_sholder_elbow >= 0: #suppongo sempre che il gomito sia piu in basso della spalla
        clock_wise_momentum_sholder = True
        non_clock_wise_momentum_sholder = False
    else:
        non_clock_wise_momentum_sholder = True
        clock_wise_momentum_sholder = False
        #print(angle_sholder_elbow)
    #print(x_sholder_elbow, y_sholder_elbow)

    before_shiftx_Rwrist = before_Rwrist[0][0] - before_Rwrist[len_before][0]
    before_vx_Rwrist = before_shiftx_Rwrist/time_before
    before_shifty_Rwrist = before_Rwrist[0][1] - before_Rwrist[len_before][1]
    before_vy_Rwrist = before_shifty_Rwrist/time_before
    before_shift_Rwrist = math.sqrt(before_shiftx_Rwrist**2+before_shifty_Rwrist**2)
    before_v_Rwrist = before_shift_Rwrist/time_before
    before_alpha_Rwrist = math.degrees(math.acos(before_vx_Rwrist/before_v_Rwrist))
    before_w_Rwrist = before_alpha_Rwrist/time_before
    print("Rwirst x,y,tot,v\n", before_shiftx_Rwrist, before_shifty_Rwrist, before_shift_Rwrist, before_v_Rwrist, before_w_Rwrist, before_vx_Rwrist)

    x_elbow_wrist = before_Relbow[len_before][0]-before_Rwrist[len_before][0] #inteoria momento dell'impatto
    print(x_elbow_wrist)
    if x_elbow_wrist < l_forearm: #significa che la linea gomito polso non e parallela a x, il problema e che introduco un sacco di approximation
        theta = math.degrees(math.acos(x_elbow_wrist/l_forearm))
        print(theta)
        #z_elbow_wrist =
    if x_elbow_wrist > 0:
        y_elbow_wrist = before_Relbow[len_before][1]-before_Rwrist[len_before][1]
        forearm_pixel = math.sqrt(x_elbow_wrist**2+y_elbow_wrist**2)
        #if y_elbow_wrist >= 0: #elbow piu "basso"
        #    angle_elbow_wrist = math.degrees(math.acos(x_elbow_wrist/forearm_pixel))
        #else:
        angle_elbow_wrist = math.degrees(math.acos(x_elbow_wrist/forearm_pixel))

    len_load_2_hit = len(load_2_hit)-1
    print(len_load_2_hit, load_2_hit[0][0]-load_2_hit[len_load_2_hit][0])
    acceleration = (before_vx_Rwrist)/((len_load_2_hit+1)/fps)
    beginning_force = acceleration*(player_hand+player_arm+player_forearm+raquet_mass)
    print(before_vx_Rwrist, acceleration, beginning_force)

    #print(x_elbow_wrist, y_elbow_wrist, angle_elbow_wrist)
    #print(math.degrees(math.acos(x_elbow_wrist/forearm_pixel)))
    #after the impact the points decelerate
    len_after = len(after_Rsholder)-1
    #print(len_after)
    time_after = (len_after+1)/fps
    #print(time_after)
    print("dopo 30frames")

    after_shiftx_Rsholder = math.fabs(after_Rsholder[0][0] - after_Rsholder[len_after][0])
    after_vx_Rsholder = after_shiftx_Rsholder/time_after
    after_shifty_Rsholder = after_Rsholder[0][1] - after_Rsholder[len_after][1]
    after_vy_Rsholder = after_shifty_Rsholder/time_after
    after_shift_Rsholder = math.sqrt(after_shiftx_Rsholder**2+after_shifty_Rsholder**2)
    after_v_Rsholder = after_shift_Rsholder/time_after
    after_alpha_Rsholder = math.degrees(math.acos(after_vx_Rsholder/after_v_Rsholder))
    after_w_Rsholder = after_alpha_Rsholder/time_after
    print("Rsholder x,y,tot,v\n", after_shiftx_Rsholder, after_vx_Rsholder, after_shifty_Rsholder, after_vy_Rsholder, after_shift_Rsholder, after_v_Rsholder, after_w_Rsholder)

    after_shiftx_Relbow = after_Relbow[0][0] - after_Relbow[len_after][0]
    after_vx_Relbow = after_shiftx_Relbow/time_after
    after_shifty_Relbow = after_Relbow[0][1] - after_Relbow[len_after][1]
    after_vy_Relbow = after_shifty_Relbow/time_after
    after_shift_Relbow = math.sqrt(after_shiftx_Relbow**2+after_shifty_Relbow**2)
    after_v_Relbow = after_shift_Relbow/time_after
    after_alpha_Relbow = math.degrees(math.acos(after_vx_Relbow/after_v_Relbow))
    after_w_Relbow = after_alpha_Relbow/time_after
    print("Relbow x,y,tot,v\n", after_shiftx_Relbow, after_shifty_Relbow, after_shift_Relbow, after_v_Relbow, after_w_Relbow)

    after_shiftx_Rwrist = after_Rwrist[0][0] - after_Rwrist[len_after][0]
    after_vx_Rwrist = after_shiftx_Rwrist/time_after
    after_shifty_Rwrist = after_Rwrist[0][1] - after_Rwrist[len_after][1]
    after_vy_Rwrist = after_shifty_Rwrist/time_after
    after_shift_Rwrist = math.sqrt(after_shiftx_Rwrist**2+after_shifty_Rwrist**2)
    after_v_Rwrist = after_shift_Rwrist/time_after
    after_alpha_Rwrist = math.degrees(math.acos(after_vx_Rwrist/after_v_Rwrist))
    after_w_Rwrist = after_alpha_Rwrist/time_after
    print("Rwirst x,y,tot,v\n", after_shiftx_Rwrist, after_shifty_Rwrist, after_shift_Rwrist, after_v_Rwrist, after_w_Rwrist, after_vx_Rwrist)


    if after_w_Rsholder<before_w_Rsholder:
        print("sholder deceleration!")
    if after_w_Relbow<before_w_Relbow:
        print("elbow deceleration!")
    if after_w_Rwrist<before_w_Rwrist:
        print("wrist deceleration!")


    forces_i = []
    moments_i = []
    raquet = True
    wrist = False
    elbow = False
    sholder = False


    def recoursive_eq_solver(force):

    #bisogna verificare il sistema di riferimento di ogni punto, perche le forze ed i momenti possono cambiare significativamente
    #ad esempio supponendo che ad ogni colpo la racchetta sia circa a 90 gradi con l'avambraccio suppongo anche che sia parallela
    #all'asse z visto che non riesco ad identificarla, pero dovrei tenerne conto. Un altro esempio e l'angolo braccio-avambraccio
    #se i punti spalla e gomito sono attraversati da una retta parallela all asse y, l'angolo calcolato e quello giusto, altrimenti
    #bisogna tenere conto dell'angolo rispetto all'asse y
    # la forza è sempre la stessa che posso traslare in tutte le direzioni, basta considerare i momenti. Inoltre, considero sempre
    # tutto in equilibrio.

        global forces_i, moments_i
        global raquet, wrist, elbow, sholder
        global clock_wise_momentum_sholder, non_clock_wise_momentum_sholder

        components_moments = [] #evry time redifined
        components_forces = [] #evry time redifined

        #raquet
        if raquet == True: #la racchetta non ha giunzioni quindi niente momenti
            bindingforce_raquet_eqx = -force
            bindingforce_raquet_eqy = -raquet_mass*gravity #raquet mass is supported by the wrist
            bindingforce_raquet_eqz = 0
            bindingmoment_raquet_eqy = 0
            bindingmoment_raquet_eqx = 0
            bindingmoment_raquet_eqz = 0

            print(bindingforce_raquet_eqx, bindingmoment_raquet_eqz)
            #print(math.sin(100))
            components_forces.append((bindingforce_raquet_eqx, bindingforce_raquet_eqy, bindingforce_raquet_eqz))
            components_moments.append((bindingmoment_raquet_eqx, bindingmoment_raquet_eqy, bindingmoment_raquet_eqz))
            wrist = True
            raquet = False

            recoursive_eq_solver(force)

        #wrist
        if wrist == True:
            bindingforce_wrist_eqx = -force #dato che il polso non si piega
            bindingforce_wrist_eqy = -(raquet_mass+player_hand)*gravity
            bindingforce_wrist_eqz = 0
            bindingmoment_wrist_eqx = bindingforce_wrist_eqy*l_impact_wrist #this depend how much the wrist is rotated in side(x), estimated 90 degrees
            bindingmoment_wrist_eqy = -l_impact_wrist*bindingforce_wrist_eqx*math.sin(angle_wrist_raquet*degrees_2_radiants)
            bindingmoment_wrist_eqz = 0
            print(bindingmoment_wrist_eqx, bindingmoment_wrist_eqy)
            #print(math.sin(100))
            components_forces.append((bindingforce_wrist_eqx, bindingforce_wrist_eqy, bindingforce_wrist_eqz))
            components_moments.append((bindingmoment_wrist_eqx, bindingmoment_wrist_eqy, bindingmoment_wrist_eqz))
            elbow = True
            wrist = False
            #print()

            recoursive_eq_solver(force)

        #elbow
        if elbow == True:
            bindingforce_elbow_eqx = -force
            bindingforce_elbow_eqy = -(raquet_mass+player_hand+player_forearm)*gravity #diversi contributi ma tutte forze dirette verso il "basso"
            bindingforce_elbow_eqz = 0
            bindingmoment_elbow_eqx = 0 #zero perche il gomito ha un solo angolo di flessione
            bindingmoment_elbow_eqy = 0
            bindingmoment_elbow_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist)*degrees_2_radiants)
            sholder = True
            elbow = False
            print(bindingmoment_elbow_eqz)

            recoursive_eq_solver(force)

        #sholder
        if sholder == True:
            bindingforce_sholder_eqx = -force
            bindingforce_sholder_eqy = -(raquet_mass+player_hand+player_forearm+player_arm)*gravity
            bindingforce_sholder_eqz = 0
            bindingmoment_sholder_eqx = 0
            bindingmoment_sholder_eqy = -l_impact_wrist*bindingforce_sholder_eqx*math.sin(angle_wrist_raquet*degrees_2_radiants) #ferma la rotazione dovuta all-impatto con la racchetta #math.sin() da sempre valore positivo
            if clock_wise_momentum_sholder == True:                                                                              #sarebbeda aggiungere la rotazione rispetto a y della spalla perche i punti gomito-polso se ruotati aggiungono leva
                bindingmoment_sholder_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist)*degrees_2_radiants) - 1/2*l_arm*player_arm*gravity*math.sin((angle_sholder_elbow)*degrees_2_radiants)#la stessa del gomito se i punti spalla e gomito sono paralleli a y
            elif non_clock_wise_momentum_sholder == True:
                bindingmoment_sholder_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist)*degrees_2_radiants) + l_arm*player_arm*gravity*math.sin((angle_sholder_elbow)*degrees_2_radiants)
            sholder = False
            print(bindingmoment_sholder_eqy, bindingmoment_sholder_eqz)
            sholder_force = math.sqrt(bindingforce_sholder_eqx**2+bindingforce_sholder_eqy**2)
            sholder_moment = math.sqrt(bindingmoment_sholder_eqy**2+bindingmoment_sholder_eqz**2)
            #data = [sholder_force, sholder_moment]
            #df = pd.DataFrame([data], columns = ['Sholder force', 'Sholder moment'])

            y1 = [sholder_force, sholder_force]
            x1 = [0.000, 0.2]
            plt.plot(x1, y1, label = "force")

            y2 = [sholder_moment, sholder_moment]
            x2 = [0.000, 0.2]
            plt.plot(x2, y2, label = "moment")

            plt.xlabel('time [n fps/fps]')
            plt.ylabel('force [pixel*kg/s^2] & moment [pixel^2*kg/s^2]')
            plt.title('Sholder force and moment')
            plt.legend()
            plt.show()

            recoursive_eq_solver(force)



    recoursive_eq_solver(46.8)


if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", required=True, help="path to input video")
    ap.add_argument("-a", "--img_average", required=True, help="path to image of video average")
    ap.add_argument("-o", "--output", help="path to output video")
    ap.add_argument("-b", "--background", help="background subtraction method (MOG2, KNN)", default="MOG2")
    args = vars(ap.parse_args())

    cap = cv.VideoCapture(args["input"])
    if cap.isOpened() != True:
        print("[ERROR] {} is an invalid video!".format(args["input"]))
    else:
        width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        print("Width " + str(width))
        print("Height " + str(height))
        fps = float(cap.get(cv.CAP_PROP_FPS))
        print("FPS", fps)
        video_frames = cap.get(cv.CAP_PROP_FRAME_COUNT)
        print("N frame video " + str(video_frames))
        video_format = cap.get(cv.CAP_PROP_FOURCC)
        print("Video format ", end = '')
        print(video_format)

        average_sub = cv.imread(args["img_average"])

        if args["background"] == "MOG2":
            back_sub = cv.createBackgroundSubtractorMOG2(500, 16, False) #MOG2 is pretty better; default but False shadows
        elif args["background"] == "KNN":
            back_sub = cv.createBackgroundSubtractorKNN()
        else:
            print("{} isn't a valid argument".format(args["background"]))

        precedent_x = 0
        traject_velocity_right = 0
        traject_velocity_left = 0

        while True: #while are still frame to read

            res, frame = cap.read()
            if res == False:
                break
            frames_passed += 1

            #cv.imshow("frame", frame)

            start_1 = time.time()

            #section for player detection
            player_mask = background_substractor(frame, average_sub)
            #cv.imshow('player_mask', player_mask)
            player, left_most = player_tracking(player_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            #section for ball detection
            """
            frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV) #not so valid a yellow detection, neither in rallenty one
            lower = np.array([22, 93, 0], dtype="uint8")
            upper = np.array([45, 255, 255], dtype="uint8")
            yellow_stuff = cv.inRange(frame_HSV, lower, upper)
            yellow_stuff = cv.morphologyEx(yellow_stuff, cv.MORPH_OPEN, np.ones((3,3),np.uint8), iterations = 1)
            yellow_contours, _ = cv.findContours(yellow_stuff, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
            cv.imshow("yellow_stuff", yellow_stuff)
            """
            ball_mask = back_sub.apply(frame)
            ball = ball_tracking(ball_mask, frame, cv.RETR_TREE, cv.CHAIN_APPROX_NONE) #RETR_TREE and CHAIN_APPROX_NONE seem the best for the ball

            #section for force calculator
            precedent_x = direction(precedent_x, left_most, start_1)
            traject_velocity_right, traject_velocity_left = velocity(fps, traject_velocity_right, traject_velocity_left)
            force(traject_velocity_right, traject_velocity_left)

            frame = cv.circle(frame, (1057,439), 2, (255,255,255)) #124
            frame = cv.circle(frame, (786,503), 2, (255,255,255)) #326
            cv.imshow("punto di carica",frame)

            if cv.waitKey(1) & 0xFF == ord('q'): #press 'q' to break loop and exit
                break


    cap.release()
    cv.destroyAllWindows()
