import os, json
import cv2 as cv
import math
# import pandas and matplotlib
import pandas as pd
import matplotlib.pyplot as plt

fps = 25

I_rigidbody = 1/3
l_impact_wrist = 0.45 #pixels 45cm
player_mass = 70
player_hand = player_mass/100 * 0.612
player_arm = player_mass/100 * 2.64
player_forearm = player_mass/100 * 1.531
player_height = 1.8 #pixels mesured
const = 341.6 #615/1.8m
t_const = 10
l_forearm = player_height/100 * 14.5 #89 pixel
l_arm = player_height/100 * 18.8
raquet_mass = 0.3
gravity = 9.81
degrees_2_radiants = (math.pi*2)/360
clock_wise_momentum_sholder = False
non_clock_wise_momentum_sholder = False
angle_wrist_raquet = 100
beginning_force = 0
decreased_force = 0
theta_elbow_wrist = 0

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
        if int(index) > 310 and int(index) < 327:
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
        if int(index) > 326 and int(index) < 358:
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
time_before = (len_before+1)/fps/t_const


#print(time_before)
print("prima 15frames")

before_shiftx_Rsholder = before_Rsholder[0][0] - before_Rsholder[len_before][0]
before_vx_Rsholder = before_shiftx_Rsholder/time_before/const
before_shifty_Rsholder = before_Rsholder[0][1] - before_Rsholder[len_before][1]
before_vy_Rsholder = before_shifty_Rsholder/time_before/const
before_shift_Rsholder = math.sqrt(before_shiftx_Rsholder**2+before_shifty_Rsholder**2)
before_v_Rsholder = before_shift_Rsholder/time_before/const
before_alpha_Rsholder = math.degrees(math.acos(before_vx_Rsholder/before_v_Rsholder))
before_w_Rsholder = before_alpha_Rsholder/time_before
print("Rsholder x,y,tot,v\n", before_shiftx_Rsholder, before_vx_Rsholder, before_shifty_Rsholder, before_vy_Rsholder, before_shift_Rsholder, before_v_Rsholder, before_alpha_Rsholder, before_w_Rsholder)

#devo verificare al momento dell'impatto com e posizionata la retta spalla gomito

before_shiftx_Relbow = before_Relbow[0][0] - before_Relbow[len_before][0]
before_vx_Relbow = before_shiftx_Relbow/time_before/const
before_shifty_Relbow = before_Relbow[0][1] - before_Relbow[len_before][1]
before_vy_Relbow = before_shifty_Relbow/time_before/const
before_shift_Relbow = math.sqrt(before_shiftx_Relbow**2+before_shifty_Relbow**2)
before_v_Relbow = before_shift_Relbow/time_before/const
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
before_vx_Rwrist = before_shiftx_Rwrist/time_before/const
before_shifty_Rwrist = before_Rwrist[0][1] - before_Rwrist[len_before][1]
before_vy_Rwrist = before_shifty_Rwrist/time_before/const
before_shift_Rwrist = math.sqrt(before_shiftx_Rwrist**2+before_shifty_Rwrist**2)
before_v_Rwrist = before_shift_Rwrist/time_before/const
before_alpha_Rwrist = math.degrees(math.acos(before_vx_Rwrist/before_v_Rwrist))
before_w_Rwrist = before_alpha_Rwrist/time_before
print("Rwirst x,y,tot,v\n", before_shiftx_Rwrist, before_shifty_Rwrist, before_shift_Rwrist, before_v_Rwrist, before_w_Rwrist, before_vx_Rwrist)

x_elbow_wrist = before_Relbow[len_before][0]-before_Rwrist[len_before][0] #inteoria momento dell'impatto
#y_elbow_wrist = before_Relbow[len_before][1]-before_Rwrist[len_before][1]
print(x_elbow_wrist)
if x_elbow_wrist < l_forearm*const: #significa che la linea gomito polso non e parallela a x, il problema e che introduco un sacco di approximation
    theta_elbow_wrist = math.fabs(90-math.degrees(math.acos(x_elbow_wrist/l_forearm*const)))
    print(theta_elbow_wrist)
    #z_elbow_wrist =
if x_elbow_wrist > 0:
    y_elbow_wrist = before_Relbow[len_before][1]-before_Rwrist[len_before][1]
    forearm_pixel = math.sqrt(x_elbow_wrist**2+y_elbow_wrist**2)
    #if y_elbow_wrist >= 0: #elbow piu "basso"
    #    angle_elbow_wrist = math.degrees(math.acos(x_elbow_wrist/forearm_pixel))
    #else:
    angle_elbow_wrist_y = math.degrees(math.acos(x_elbow_wrist/forearm_pixel)) #rotazione su y
print(angle_elbow_wrist_y)
len_load_2_hit = len(load_2_hit)-1
print(len_load_2_hit, load_2_hit[0][0]-load_2_hit[len_load_2_hit][0])
acceleration = (before_v_Rwrist)/((len_load_2_hit+1)/fps*10)
beginning_force = acceleration*(player_hand+player_arm+player_forearm+raquet_mass)
print(before_vx_Rwrist, acceleration, beginning_force)

#print(x_elbow_wrist, y_elbow_wrist, angle_elbow_wrist)
#print(math.degrees(math.acos(x_elbow_wrist/forearm_pixel)))
#after the impact the points decelerate
len_after = len(after_Rsholder)-1
#print(len_after)
time_after = (len_after+1)/fps/t_const
#print(time_after)
print("dopo 30frames")

after_shiftx_Rsholder = math.fabs(after_Rsholder[0][0] - after_Rsholder[len_after][0])
after_vx_Rsholder = after_shiftx_Rsholder/time_after/const
after_shifty_Rsholder = after_Rsholder[0][1] - after_Rsholder[len_after][1]
after_vy_Rsholder = after_shifty_Rsholder/time_after/const
after_shift_Rsholder = math.sqrt(after_shiftx_Rsholder**2+after_shifty_Rsholder**2)
after_v_Rsholder = after_shift_Rsholder/time_after/const
after_alpha_Rsholder = math.degrees(math.acos(after_vx_Rsholder/after_v_Rsholder))
after_w_Rsholder = after_alpha_Rsholder/time_after
print("Rsholder x,y,tot,v\n", after_shiftx_Rsholder, after_vx_Rsholder, after_shifty_Rsholder, after_vy_Rsholder, after_shift_Rsholder, after_v_Rsholder, after_w_Rsholder)

after_shiftx_Relbow = after_Relbow[0][0] - after_Relbow[len_after][0]
after_vx_Relbow = after_shiftx_Relbow/time_after/const
after_shifty_Relbow = after_Relbow[0][1] - after_Relbow[len_after][1]
after_vy_Relbow = after_shifty_Relbow/time_after/const
after_shift_Relbow = math.sqrt(after_shiftx_Relbow**2+after_shifty_Relbow**2)
after_v_Relbow = after_shift_Relbow/time_after/const
after_alpha_Relbow = math.degrees(math.acos(after_vx_Relbow/after_v_Relbow))
after_w_Relbow = after_alpha_Relbow/time_after
print("Relbow x,y,tot,v\n", after_shiftx_Relbow, after_shifty_Relbow, after_shift_Relbow, after_v_Relbow, after_w_Relbow)

after_shiftx_Rwrist = after_Rwrist[0][0] - after_Rwrist[len_after][0]
after_vx_Rwrist = after_shiftx_Rwrist/time_after/const
after_shifty_Rwrist = after_Rwrist[0][1] - after_Rwrist[len_after][1]
after_vy_Rwrist = after_shifty_Rwrist/time_after/const
after_shift_Rwrist = math.sqrt(after_shiftx_Rwrist**2+after_shifty_Rwrist**2)
after_v_Rwrist = after_shift_Rwrist/time_after/const
after_alpha_Rwrist = math.degrees(math.acos(after_vx_Rwrist/after_v_Rwrist))
after_w_Rwrist = after_alpha_Rwrist/time_after
print("Rwirst x,y,tot,v\n", after_shiftx_Rwrist, after_shifty_Rwrist, after_shift_Rwrist, after_v_Rwrist, after_w_Rwrist, after_vx_Rwrist)

deceleration_wrist = math.fabs(before_v_Rwrist-after_v_Rwrist)/time_after
deceleration_elbow = math.fabs(before_v_Relbow-after_v_Relbow)/time_after
deceleration_sholder = math.fabs(before_v_Rsholder-after_v_Rsholder)/time_after
decreased_force_sholder = deceleration_sholder * (player_hand+player_arm+player_forearm+raquet_mass)
decreased_force_elbow = deceleration_elbow * (player_hand+player_forearm+raquet_mass)
decreased_force_wrist = deceleration_wrist * (player_hand+raquet_mass)

print(before_v_Relbow, after_v_Relbow, deceleration_sholder, time_after)

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

wrist_force, elbow_force, sholder_force = 0,0,0
wrist_force_beginning, elbow_force_beginning, sholder_force_beginning = 0,0,0
wrist_moment, elbow_moment, sholder_moment = 0,0,0
wrist_moment_beginning, elbow_moment_beginning, sholder_moment_beginning = 0,0,0


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
    global beginning_force
    global wrist_force, elbow_force, sholder_force
    global wrist_force_beginning, elbow_force_beginning, sholder_force_beginning
    global wrist_moment, elbow_moment, sholder_moment
    global wrist_moment_beginning, elbow_moment_beginning, sholder_moment_beginning
    global theta_elbow_wrist

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
        wrist_force = math.sqrt(bindingforce_wrist_eqx**2+bindingforce_wrist_eqy**2)
        wrist_force_beginning = math.fabs(bindingforce_wrist_eqy)
        wrist_moment = math.sqrt(bindingmoment_wrist_eqx**2+bindingmoment_wrist_eqy**2)
        wrist_moment_beginning = math.fabs(bindingmoment_wrist_eqx)
        #print()

        recoursive_eq_solver(force)

    #elbow
    if elbow == True:
        bindingforce_elbow_eqx = -force
        bindingforce_elbow_eqy = -(raquet_mass+player_hand+player_forearm)*gravity #diversi contributi ma tutte forze dirette verso il "basso"
        bindingforce_elbow_eqz = 0
        bindingmoment_elbow_eqx = 0 #zero perche il gomito ha un solo angolo di flessione
        bindingmoment_elbow_eqy = 0

        if angle_elbow_wrist_y == 0: #linea elbow-wrist parallela a x
            bindingmoment_elbow_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity
        else:
            bindingmoment_elbow_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist_y)*degrees_2_radiants) +bindingforce_elbow_eqx*l_forearm*math.sin((angle_elbow_wrist_y)*degrees_2_radiants)
        sholder = True
        elbow = False
        print(bindingmoment_elbow_eqz)
        elbow_force = math.sqrt(bindingforce_elbow_eqx**2+bindingforce_elbow_eqy**2)
        elbow_force_beginning = math.fabs(bindingforce_elbow_eqy)
        elbow_moment = math.fabs(bindingmoment_elbow_eqz)
        elbow_moment_beginning = math.fabs(l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity)

        recoursive_eq_solver(force)

    #sholder
    if sholder == True:
        bindingforce_sholder_eqx = -force
        bindingforce_sholder_eqy = -(raquet_mass+player_hand+player_forearm+player_arm)*gravity
        bindingforce_sholder_eqz = 0
        bindingmoment_sholder_eqx = 0
        print(theta_elbow_wrist)
        if theta_elbow_wrist == 0: #se il gomito ruotasse verso l-esterno, sul piano x
            bindingmoment_sholder_eqy = -l_impact_wrist*bindingforce_sholder_eqx*math.sin(angle_wrist_raquet*degrees_2_radiants) #ferma la rotazione dovuta all-impatto con la racchetta #math.sin() da sempre valore positivo
        else:
            bindingmoment_sholder_eqy = -(l_forearm*math.sin(angle_elbow_wrist_y*degrees_2_radiants)+l_impact_wrist)*bindingforce_sholder_eqx*math.sin(angle_wrist_raquet*degrees_2_radiants)
        if angle_elbow_wrist_y == 0:
            if clock_wise_momentum_sholder == True:                                                                              #sarebbe da aggiungere la rotazione rispetto a y della spalla perche i punti gomito-polso se ruotati aggiungono leva
                bindingmoment_sholder_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist_y)*degrees_2_radiants) - 1/2*l_arm*player_arm*gravity*math.sin((angle_sholder_elbow)*degrees_2_radiants) +bindingforce_sholder_eqx*l_arm#la stessa del gomito se i punti spalla e gomito sono paralleli a y
            elif non_clock_wise_momentum_sholder == True:
                bindingmoment_sholder_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist_y)*degrees_2_radiants) + 1/2*l_arm*player_arm*gravity*math.sin((angle_sholder_elbow)*degrees_2_radiants) +bindingforce_sholder_eqx*l_arm
            sholder_moment_beginning = math.fabs(bindingmoment_sholder_eqz-bindingforce_sholder_eqx*l_arm)
        else:
            if clock_wise_momentum_sholder == True:                                                                              #sarebbe da aggiungere la rotazione rispetto a y della spalla perche i punti gomito-polso se ruotati aggiungono leva
                bindingmoment_sholder_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist_y)*degrees_2_radiants) - 1/2*l_arm*player_arm*gravity*math.sin((angle_sholder_elbow)*degrees_2_radiants) +bindingforce_sholder_eqx*(l_forearm*math.sin((angle_elbow_wrist_y)*degrees_2_radiants)+l_arm)
            elif non_clock_wise_momentum_sholder == True:
                bindingmoment_sholder_eqz = -l_forearm*(raquet_mass+player_hand + 1/2*player_forearm)*gravity * math.sin((90-angle_elbow_wrist_y)*degrees_2_radiants) + 1/2*l_arm*player_arm*gravity*math.sin((angle_sholder_elbow)*degrees_2_radiants) +bindingforce_sholder_eqx*(l_forearm*math.sin((angle_elbow_wrist_y)*degrees_2_radiants)+l_arm)
            sholder_moment_beginning = math.fabs(bindingmoment_sholder_eqz-bindingforce_sholder_eqx*l_arm)
        sholder = False
        print(bindingmoment_sholder_eqz, bindingmoment_sholder_eqy)
        sholder_force = math.sqrt(bindingforce_sholder_eqx**2+bindingforce_sholder_eqy**2)
        sholder_force_beginning = math.fabs(bindingforce_sholder_eqy)
        sholder_moment = math.fabs(math.sqrt(bindingmoment_sholder_eqy**2+bindingmoment_sholder_eqz**2))

        #data = [sholder_force, sholder_moment]
        #df = pd.DataFrame([data], columns = ['Sholder force', 'Sholder moment'])
        print(sholder_force)
        """
        y1 = [0, sholder_force_beginning, sholder_force_beginning, sholder_force, sholder_force, decreased_force_sholder, 0]
        x1 = [0, 0, 0.812, 0.812, 0.817, 0.941, 1.3]
        plt.plot(x1, y1, label = "shoulder forces")



        y2 = [0, elbow_force_beginning, elbow_force_beginning, elbow_force, elbow_force, decreased_force_elbow, 0]
        x2 = [0, 0, 0.812, 0.812, 0.817, 0.941, 1.3]
        plt.plot(x2, y2, label = "elbow forces")

        y3 = [0, wrist_force_beginning, wrist_force_beginning, wrist_force, wrist_force, decreased_force_wrist, 0]
        x3 = [0, 0, 0.812, 0.812, 0.817, 0.941, 1.3]
        plt.plot(x3, y3, label = "wrist forces")

        plt.xlabel('time [s]')
        plt.ylabel('force [m*kg/s^2]')
        plt.title('Forces')
        plt.legend()
        #plt.show()
        plt.savefig('forces.png')

        """
        y1 = [0, sholder_moment_beginning, sholder_moment_beginning, sholder_moment, sholder_moment, sholder_moment_beginning, 0]
        x1 = [0, 0, 0.812, 0.812, 0.817, 0.941, 1.3]
        plt.plot(x1, y1, label = "shoulder moments")



        y3 = [0, elbow_moment_beginning, elbow_moment_beginning, elbow_moment, elbow_moment, elbow_moment_beginning, 0]
        x3 = [0, 0, 0.812, 0.812, 0.817, 0.941, 1.3]
        plt.plot(x3, y3, label = "elbow moments")

        y2 = [0, wrist_moment_beginning, wrist_moment_beginning, wrist_moment, wrist_moment, wrist_moment_beginning, 0]
        x2 = [0, 0, 0.812, 0.812, 0.817, 0.941, 1.3]
        plt.plot(x2, y2, label = "wrist moments")


        plt.xlabel('time [s]')
        plt.ylabel('moment [N*m]')
        plt.title('Moments')
        plt.legend()
        #plt.show()
        plt.savefig('moments.png')

        recoursive_eq_solver(force)



recoursive_eq_solver(54.8) #54.8*341.6
