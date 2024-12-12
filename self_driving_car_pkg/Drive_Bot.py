from .Detection.Lanes.Lane_Detection import detect_Lane
from .Detection.Signs.SignDetectionApi import detect_Signs
from .Detection.TrafficLights.TrafficLights_Detection import detect_TrafficLights
import cv2
from numpy import interp
from .config import config
from rclpy.node import Node
from geometry_msgs.msg import Twist

#############################
from .ObstacleDetect import set_mode
import time

# 4 Improvements that will be done in (Original) SDC control algorithm
# a) lane assist had iregular steering predictions
#    Solution : use rolling average filter
# b) Considering road is barely 1.5 car wide. A quarter of Image width for distance from the road mid
#                                             from the predicted road center seems bit too harsh
#    Solution:  Increase to half of image width
# c) Car was drifting offroad in sharper turns causing it to lose track of road
#    Solution: Increase weightage of distance (road_center <=> car front) from 50% to 65%
#              So steers more in case it drift offroad
# d) Car not utilizing its full steering range causing it to drift offroad in sharp turns
#    Solution: Increase car max turn capability

# 2 additons to Drive_Bot.py
# a) 1 control block added for enable/disable Sat_Nav feature
# b) Track Traffic Light and Road Speed Limits (State)  ==> Essential for priority control mechanism
#                                                           That we will create for integrating Sat_Nav
#                                                           ability to the SDC


from collections import deque
class Debugging:

    def __init__(self):
        self.TL_Created = False
        self.Lan_Created = False

    def nothing(self,x):
        pass

    cv2.namedWindow('CONFIG')

    enable_SatNav = 'Sat-Nav'
    cv2.createTrackbar(enable_SatNav, 'CONFIG',False,True,nothing)

    # creating (Engine) on/off trackbar
    Motors = 'Engine'
    cv2.createTrackbar(Motors, 'CONFIG',False,True,nothing)

    # create switch for ON/OFF functionality
    debugging_SW = 'Debug'
    cv2.createTrackbar(debugging_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingLane_SW = 'Debug Lane'
    cv2.createTrackbar(debuggingLane_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingSigns_SW = 'Debug Sign'
    cv2.createTrackbar(debuggingSigns_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingTL_SW = 'Debug TL'
    cv2.createTrackbar(debuggingTL_SW, 'CONFIG',False,True,nothing)


    def setDebugParameters(self):
        # get current positions of four trackbars
        # get current positions of trackbar
        # get current positions of four trackbars
        enable_SatNav = cv2.getTrackbarPos(self.enable_SatNav,'CONFIG')
        Motors = cv2.getTrackbarPos(self.Motors,'CONFIG')

        debug = cv2.getTrackbarPos(self.debugging_SW,'CONFIG')
        debugLane = cv2.getTrackbarPos(self.debuggingLane_SW,'CONFIG')
        debugSign = cv2.getTrackbarPos(self.debuggingSigns_SW,'CONFIG')
        debugTrafficLights = cv2.getTrackbarPos(self.debuggingTL_SW,'CONFIG')


        if enable_SatNav:
            config.enable_SatNav = True
        else:
            config.enable_SatNav = False

        # If trackbar changed modify engines_on config parameter
        if Motors:
            config.engines_on = True
        else:
            config.engines_on = False

        if debug:
            config.debugging = True
        else:
            config.debugging = False
        if debugLane:
            config.debugging_Lane = True
        else:
            config.debugging_Lane = False
        if debugSign:
            config.debugging_Signs = True
        else:
            config.debugging_Signs = False
        if debugTrafficLights:
            config.debugging_TrafficLights = True
        else:
            config.debugging_TrafficLights = False

        if config.debugging_TrafficLights:

            debuggingTLConfig_SW = 'Debug Config'
            if not self.TL_Created:
                self.TL_Created = True
                cv2.namedWindow('CONFIG_TL')
                cv2.createTrackbar(debuggingTLConfig_SW, 'CONFIG_TL',False,True,self.nothing)

            debugTL_Config = cv2.getTrackbarPos(debuggingTLConfig_SW,'CONFIG_TL')

            if debugTL_Config:
                config.debugging_TL_Config = True
            else:
                config.debugging_TL_Config = False

        else:
            self.TL_Created = False
            cv2.destroyWindow('CONFIG_TL')


        if config.debugging_Lane:

            debuggingLANEConfig_SW = 'Debug (Stage)'
            if not self.Lan_Created:
                self.Lan_Created = True
                cv2.namedWindow('CONFIG_LANE')
                cv2.createTrackbar(debuggingLANEConfig_SW, 'CONFIG_LANE',0,3,self.nothing)

            debugLane_Config = cv2.getTrackbarPos(debuggingLANEConfig_SW,'CONFIG_LANE')

            if debugLane_Config == 0:
                config.debugging_L_ColorSeg = True
                config.debugging_L_Est = config.debugging_L_Cleaning = config.debugging_L_LaneInfoExtraction = False
            elif debugLane_Config == 1:
                config.debugging_L_Est = True
                config.debugging_L_ColorSeg = config.debugging_L_Cleaning = config.debugging_L_LaneInfoExtraction = False
            elif debugLane_Config == 2:
                config.debugging_L_Cleaning = True
                config.debugging_L_ColorSeg = config.debugging_L_Est = config.debugging_L_LaneInfoExtraction = False
            elif debugLane_Config == 3:
                config.debugging_L_LaneInfoExtraction = True
                config.debugging_L_ColorSeg = config.debugging_L_Est = config.debugging_L_Cleaning = False

        else:
            self.Lan_Created = False
            cv2.destroyWindow('CONFIG_LANE')

class Control:

    def __init__(self):
        self.prev_Mode = "Detection"


        self.prev_Mode_LT = "Detection"
        self.car_speed = 150
        self.angle_of_car = 0

        self.Right_turn_iterations = 0
        self.Frozen_Angle = 0

        self.right_turn_detected_time = None
        self.Detected_RightTurn = False
        self.Activat_RightTurn = False

        self.TrafficLight_iterations = 0
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False

        # [NEW]: Deque member variable created for emulating rolling average filter to get smoothed Lane's ASsist
        self.angle_queue = deque(maxlen=10)

        ##################################3
        self.prev_Mode_Ob = "Detection"
        self.Detected_avoidance = False
        self.Activat_avoidance = False
        self.Frozen_Angle_2 = 0
        self.avoidance_iterations = 0

        self.obstacle_stop_time = None  # 장애물 감지 후 정지 시작 시간
        self.stop_duration = 5  # 정지 유지 시간 (초)


    def follow_Lane(self,Max_Sane_dist,distance,curvature , Mode , Tracked_class):

        # [NEW]: Turning at normal speed is not much of a problem in simulation
        IncreaseTireSpeedInTurns = False

        if((Tracked_class!=0) and (self.prev_Mode == "Tracking") and (Mode == "Detection")):
            if  (Tracked_class =="speed_sign_30"):
                self.car_speed = 90
            elif(Tracked_class =="speed_sign_60"):
                self.car_speed = 180
            elif(Tracked_class =="speed_sign_90"):
                self.car_speed = 270
            elif(Tracked_class =="stop"):
                self.car_speed = 0

        self.prev_Mode = Mode # Set prevMode to current Mode

        Max_turn_angle_neg = -90
        Max_turn_angle = 90

        CarTurn_angle = 0

        if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ):
            # Max sane distance reached ---> Max penalize (Max turn Tires)
            if(distance > Max_Sane_dist):
                #Car offseted left --> Turn full wheels right
                CarTurn_angle = Max_turn_angle + curvature
            else:
                #Car Offseted right--> Turn full wheels left
                CarTurn_angle = Max_turn_angle_neg + curvature
        else:
            # Within allowed distance limits for car and lane
            # Interpolate distance to Angle Range
            Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90])
            #[NEW]: Modified to calculate carturn_angle based on following criteria
            #             65% turn suggested by distance to the lane center + 35 % how much the lane is turning
            CarTurn_angle = (0.65*Turn_angle_interpolated) + (0.35*curvature)

        # Handle Max Limit [if (greater then either limits) --> set to max limit]
        if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
            if(CarTurn_angle > Max_turn_angle):
                CarTurn_angle = Max_turn_angle
            else:
                CarTurn_angle = -Max_turn_angle

        #angle = CarTurn_angle
        # [NEW]: Increase car turning capability by 30 % to accomodate sharper turns
        angle = interp(CarTurn_angle,[-90,90],[-60,60])

        curr_speed = self.car_speed

        if (IncreaseTireSpeedInTurns and (Tracked_class !="right_turn")):
            if(angle>30):
                car_speed_turn = interp(angle,[30,45],[80,100])
                curr_speed = car_speed_turn
            elif(angle<-30):
                car_speed_turn = interp(angle,[-45,-30],[100,80])
                curr_speed = car_speed_turn


        return angle , curr_speed


    def Obey_RightTurn(self, Angle, Speed, Mode, Tracked_class):
        if Tracked_class == "right_turn":  # 우회전이 감지된 경우
            Speed = 150  # 속도 고정

            if self.prev_Mode_LT == "Detection" and Mode == "Tracking":
                self.prev_Mode_LT = "Tracking"
                self.Detected_RightTurn = True

            elif self.prev_Mode_LT == "Tracking" and Mode == "Detection":
                # 우회전 감지가 완료되었을 때
                self.Detected_RightTurn = False
                self.Activat_RightTurn = True

                if self.Right_turn_iterations % 10 == 0 and self.Right_turn_iterations > 30:
                    self.Frozen_Angle = self.Frozen_Angle + 32 # 우회전 각도 추가
                if self.Right_turn_iterations == 110:  # 우회전 종료 조건
                    self.prev_Mode_LT = "Detection"
                    self.Activat_RightTurn = False
                    self.Right_turn_iterations = 0
                    self.Frozen_Angle = 0

                self.Right_turn_iterations += 1

                if self.Activat_RightTurn:
                    Angle = self.Frozen_Angle  # 우회전 각도 설정

        return Angle, Speed, self.Detected_RightTurn, self.Activat_RightTurn
    
    def OBEY_TrafficLights(self,a,b,Traffic_State,CloseProximity):

        if((Traffic_State == "Stop") and CloseProximity):
            b = 0 # Noob luqman
            self.STOP_MODE_ACTIVATED = True
        else:
            if (self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED):

                if (self.STOP_MODE_ACTIVATED and (Traffic_State=="Go")):
                    self.STOP_MODE_ACTIVATED = False
                    self.GO_MODE_ACTIVATED = True

                elif(self.STOP_MODE_ACTIVATED):
                    b = 0

                elif(self.GO_MODE_ACTIVATED):
                    a = 0.0
                    if(self.TrafficLight_iterations==200):
                        self.GO_MODE_ACTIVATED = False
                        # print("Interchange Crossed !!!")
                        self.TrafficLight_iterations = 0 #Reset

                    self.TrafficLight_iterations = self.TrafficLight_iterations + 1
        return a,b

    # ##########################################

    def obstacle_avoidance(self, Angle, Speed, Det_mode, Det_class):
        if Det_class == "Avoidance":
            if self.prev_Mode_Ob == "Detection" and Det_mode == "Tracking":
                Speed = 0  # 장애물 감지 시 우선 정지
                self.prev_Mode_Ob = "Tracking"
                self.Detected_avoidance = True
                self.obstacle_stop_time = time.time()  # 정지 시작 시간 기록
                print("[Obstacle Avoidance] Obstacle detected. Stopping and preparing for avoidance.")

            elif self.prev_Mode_Ob == "Tracking" and Det_mode == "Detection":
                self.Detected_avoidance = False
                self.Activat_avoidance = True

                # 회피 동작 (좌측으로 이동)
                if self.avoidance_iterations < 50:
                    self.Frozen_Angle_2 -= 7  # 좌측으로 이동
                    Speed = 90  # 속도 설정
                    print(f"[Obstacle Avoidance] Executing avoidance maneuver. Iteration: {self.avoidance_iterations}, Angle: {self.Frozen_Angle_2}, Speed: {Speed}")
                elif self.avoidance_iterations == 50:
                    # 회피 완료
                    self.prev_Mode_Ob = "Detection"
                    self.Activat_avoidance = False
                    self.avoidance_iterations = 0
                    self.Frozen_Angle_2 = 0
                    Det_class = "Reposition"
                    print("[Obstacle Avoidance] Avoidance complete. Switching to reposition mode.")

                self.avoidance_iterations += 1

            # 회피 동작 중 저장된 각도 유지
            if self.Activat_avoidance or self.Detected_avoidance:
                Angle = self.Frozen_Angle_2

        return Angle, Speed, Det_class




    def reposition_car(self, Angle, Speed, Det_mode, Det_class):
        if Det_class == "Reposition":
            Speed = 120  # 복귀 속도 설정

            if self.prev_Mode_Ob == "Detection" and Det_mode == "Tracking":
                self.prev_Mode_Ob = "Tracking"
                self.Detected_avoidance = True

            elif self.prev_Mode_Ob == "Tracking" and Det_mode == "Detection":
                self.Detected_avoidance = False
                self.Activat_avoidance = True

                # 복귀 동작 단계별 처리
                if self.avoidance_iterations < 50:
                    self.Frozen_Angle_2 += 7  # 우측으로 복귀 시작
                elif self.avoidance_iterations < 100:
                    self.Frozen_Angle_2 = 0  # 직진 유지
                elif self.avoidance_iterations < 150:
                    self.Frozen_Angle_2 += 6  # 우측 복귀 완료

                # 복귀 완료 조건
                if self.avoidance_iterations == 150:
                    self.prev_Mode_Ob = "Detection"
                    self.Activat_avoidance = False
                    self.avoidance_iterations = 0
                    self.Frozen_Angle_2 = 0
                    Det_class = "None"

                self.avoidance_iterations += 1

            # 복귀 동작 중 저장된 각도 유지
            if self.Activat_avoidance or self.Detected_avoidance:
                Angle = self.Frozen_Angle_2

        return Angle, Speed, Det_class



    
    ########################################


    def drive_car(self,Current_State,Inc_TL,Inc_LT):
        """Act on extracted information based on the SDC control mechanism

        Args:
            Current_State (List): information extracted from SDC surroundings
                                    E.g. (Information regarding the lane boundaries for lane assist +
                                        Information regarding the traffic signs for cruise control)
            Inc_TL (bool): Toggle [Intersection Navigation] ON | OFF
            Inc_LT (bool): Toggle [Obey Left Turn Sign] ON | OFF
        Returns:
            angle_of_car  (int): required steering angle for the SDC
            current_speed  (int): required cruise speed for the SDC
            Detected_RightTurn  (bool): Indicates if SDC has detected a left turn sign
            Activat_RightTurn  (bool): Indicates if SDC Take_Right_turn mechanism is activated
        """

        [Distance, Curvature, frame_disp , Mode , Tracked_class, Traffic_State, CloseProximity, Det_mode, Det_class] = Current_State

        current_speed = 0

        if((Distance != -1000) and (Curvature != -1000)):

            # [NEW]: Very Important: Minimum Sane Distance that a car can be from the perfect lane to follow is increased to half its fov.
            #                        This means sharp turns only in case where we are way of target XD
            self.angle_of_car , current_speed = self.follow_Lane(int(frame_disp.shape[1]/2), Distance,Curvature , Mode , Tracked_class )
        # [NEW]: Keeping track of orig steering angle and smoothed steering angle using rolling average
        config.angle_orig = self.angle_of_car
        
        # Rolling average applied to get smoother steering angles for robot
        self.angle_queue.append(self.angle_of_car)
        self.angle_of_car = (sum(self.angle_queue)/len(self.angle_queue))
        config.angle = self.angle_of_car
        
        #######################
        self.angle_of_car,current_speed, Det_class = self.obstacle_avoidance(self.angle_of_car, current_speed,Det_mode,Det_class)
        self.angle_of_car,current_speed, Det_class = self.reposition_car(self.angle_of_car, current_speed,Det_mode, Det_class)

        if Inc_LT:
            self.angle_of_car,current_speed, Detected_RightTurn, Activat_RightTurn = self.Obey_RightTurn(self.angle_of_car,current_speed,Mode,Tracked_class)
        else:
            Detected_RightTurn = False
            Activat_RightTurn = False

        if Inc_TL:
            self.angle_of_car,current_speed = self.OBEY_TrafficLights(self.angle_of_car,current_speed,Traffic_State,CloseProximity)


        return self.angle_of_car,current_speed, Detected_RightTurn, Activat_RightTurn, Det_class

from .ObstacleDetect import set_mode

def set_mode(range, prev_det_class):
    if range < 8:  # 특정 거리 이내에서 장애물 감지
        return "Tracking", "Avoidance"
    elif prev_det_class == "Avoidance":  # 장애물이 사라지면 복귀 상태로 전환
        return "Detection", "Reposition"
    else:
        return "Detection", "None"


class CarControl(Node):
    def __init__(self):
        super().__init__('car_control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish_control_commands(self, angle, speed):
        msg = Twist()
        msg.angular.z = float(angle)  # 스티어링 각도 설정
        msg.linear.x = float(speed)    # 속도 설정
        self.publisher.publish(msg)
        self.get_logger().info(f"Published control - Angle: {angle}, Speed: {speed}")

class Car:
    def __init__( self,Inc_TL = True, Inc_LT = True ):

        self.Control_ = Control()

        self.car_control = CarControl()
        self.Inc_TL = Inc_TL
        self.Inc_LT = Inc_LT
        # [NEW]: Containers to Keep track of current state of Signs and Traffic Light detection
        self.Tracked_class = "Unknown"
        self.Traffic_State = "Unknown"
        
        ##################
        self.Det_class = "Unknown"

    def stop(self):
        # 차량 멈추기
        angle = 0.0
        speed = 0.0
        self.car_control.publish_control_commands(angle, speed)  # 멈춤 명령 발행
        # self.person_detected = True  # 감지 상태 업데이트

    def display_state(self,frame_disp,angle_of_car,current_speed,Tracked_class,Traffic_State,Detected_RightTurn, Activat_RightTurn):

        ###################################################  Displaying CONTROL STATE ####################################

        if (angle_of_car <-10):
            direction_string="[   Left   ]"
            color_direction=(120,0,255)
        elif (angle_of_car >10):
            direction_string="[   Right  ]"
            color_direction=(120,0,255)
        else:
            direction_string="[ Straight ]"
            color_direction=(0,255,0)

        if(current_speed>0):
            direction_string = "|| DIRECTION || = " +direction_string
        else:
            color_direction=(0,0,255)


        cv2.putText(frame_disp,str(direction_string),(20,45),cv2.FONT_HERSHEY_DUPLEX,0.55,(5,25,175),1)

        angle_speed_str = "|| ANGLE  |  SPEED || = [ " + str(int(angle_of_car)) + "deg | " + str(int(current_speed)//3) + "km/h ]"
        cv2.putText(frame_disp,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.55,(163,76,3),1)

        cv2.putText(frame_disp,"|| TRAFFIC LIGHT || = [ "+Traffic_State+" ] ",(20,70),cv2.FONT_HERSHEY_DUPLEX,0.55,(163,76,3))

        if (Tracked_class=="right_turn"):
            font_Scale = 0.32
            if (Detected_RightTurn):
                Tracked_class =  " R/Turn Detected "
            else:
                Tracked_class =  " R/Turn Activated | "+ str(Activat_RightTurn)
        else:
            font_Scale = 0.37
        cv2.putText(frame_disp,"|| SIGN || = [ "+str(Tracked_class) + " ]",(20,95),cv2.FONT_HERSHEY_DUPLEX,0.55,(5,25,175),1)

     
    def driveCar(self,frame, range):

        """ Runs the complete Self Drive Mechanism in two sequential steps:
            1) Detection : Extract all the required information from the surrounding using the sensor (camera)
            2) Control   : Act on the extracted information based on the features the SDC is capable of.

        Args:
            frame (numpy nd array): Prius front-cam view
        Returns:
            Angle (float): required steering angle given the conditions
            Speed (float): required cruise speed given the conditions
            img   (numpy_nd_array): displays the self drive under-the-hood working by overlaying
        """

        img = frame[0:640,238:1042]
        img = cv2.resize(img,(480,360))

        img_orig = img.copy()

        distance, Curvature = detect_Lane(img)

        if self.Inc_TL:
            Traffic_State, CloseProximity = detect_TrafficLights(img_orig.copy(),img)
        else:
            Traffic_State = "Unknown"
            CloseProximity = False

        Mode , Tracked_class = detect_Signs(img_orig,img)

        #########################
        Det_mode, Det_class = set_mode(range, self.Det_class)

        Current_State = [distance, Curvature, img, Mode, Tracked_class, Traffic_State, CloseProximity,Det_mode, Det_class]

        Angle,Speed, Detected_RightTurn, Activat_RightTurn, Det_class  = self.Control_.drive_car(Current_State,self.Inc_TL,self.Inc_LT)

    
        # [NEW]: Updating State Variable with current state
        self.Tracked_class = Tracked_class
        self.Traffic_State = Traffic_State
        ##############################
        self.Det_class = Det_class

        self.display_state(img,Angle,Speed,Tracked_class,Traffic_State, Detected_RightTurn, Activat_RightTurn)
 
        if (range < 8):
            cv2.putText(img,"[Object detected]",(20,120),cv2.FONT_HERSHEY_SIMPLEX,0.6,(230,45,20),1)

        # [NEW]: Interpolate increased car steering range to increased motor turning angle
        # Translate [ Real World angle and speed ===>> ROS Car Control Range ]
        Angle=interp(Angle,[-60,60],[0.8,-0.8])
        if (Speed!=0):
            Speed=interp(Speed,[30,90],[1,2])

        Speed = float(Speed)

        return Angle, Speed, img