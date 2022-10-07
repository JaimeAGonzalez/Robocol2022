#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mayo 5 2022
@author: Jaime Nuñez-Yaisa Rairez
"""
# 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
#Librerias necesarias
import time
import rospy
from smbus2 import SMBus
from sensor_msgs.msg import Joy

motorSelect=0


def callback(data):
    
    #Global variable (wanted motor)
    global motorSelect
    
	#Information for each motor
    d=False
    Data1=[]
    Data2=[]
    Address=[]
    
    #Clockwise motor direction
    LD=int(data.buttons[6])
    
    #Counter-Clockwise motor direction
    RD=int(data.buttons[7])

         #Digtal Input condition: Manual Motor Control
    Motor1=int(data.buttons[1])
    Motor2=int(data.buttons[0])
    Motor3=int(data.buttons[3])
    Motor4=int(data.buttons[2])
    Motor5=int(data.buttons[4])
    Motor6=int(data.buttons[5])    

    MotorSignal=[Motor1,Motor2,Motor3,Motor4,Motor5,Motor6]
      
    #Motor selection    
    for i in MotorSignal:
    	if (i==1):
            motorSelect=MotorSignal.index(i)+1
            print("Motor "+str(motorSelect))
            break

    #Manual Control movement
    if(LD==1 or RD==1):
        if(LD==1):
            Np=-1
            direction=byteDirection(Np,motorSelect)
        elif(RD==1):
            Np=1
            direction=byteDirection(Np,motorSelect)
        d=True
        Data1.append(int(direction[0],2))
        Data2.append(int(direction[1],2))
        Address.append(address(motorSelect))

        for i in range(len(Address)):
            if (d):
                with SMBus(1) as bus:
                    step=1
                    time.sleep(0.01)
                    #Variable: Step objetivo
                    Var=0
                    while  Var<=step:
                        bus.write_byte(Address[i],Data1[i])				 
                        bus.write_byte(Address[i],Data2[i])
                        Var=Var+1
                        print("Motor "+str(motorSelect)+" moved")
        
        

    #Digtal Input condition: Manual Motor Control
    Forward=int(data.axes[7])
    Backward=int(data.buttons[9])     
    Up=int(data.buttons[10])
    Down=int(data.buttons[11])
			                                                                                                                                                
    TrajectoryArray=[Forward,Backward,Up,Down]
    
    if(Forward==1 or Backward==1 or Up==1 or Down==1):
    	#Forward Trajectory
        if(Forward==1):
    		#Parámetros incrementales: condiciones de frontera
            d=True
            step=1
            PJ1=0
            PJ2=100
            PJ3=0
            PJ4=0
            PJ5=-100
            PJ6=0
            Nstep=[PJ1,PJ2,PJ3,PJ4,PJ5,PJ6]
    			
    		#Lista de arreglos a recorrer para cada motor
            motorArray=vectorStep(Nstep,step)
    
            for i in range(len(motorArray[0])):
                for j in range(len(Nstep)):
                    #Byte según la dirección del motor
                    direction=byteDirection(Nstep[j],j+1)
    				
        			#Información provita a cada motor
                    Address.append(address(j+1))
                    Data1.append(int(direction[0],2))
                    Data2.append(int(direction[1],2))
				
        for i in range(len(Address)):
            if (d):
                with SMBus(1) as bus:
                    #Variable: Step objetivo
                    Var=0
                    while  Var<=step:
                        bus.write_byte(Address[i],Data1[i])				 
                        bus.write_byte(Address[i],Data2[i])
                        Var=Var+1
                        print("Running forward trajectory")
				

	
def joy_listener():
	#Anonymous=True is a flag that enable rospy to choose an unique 'listener'
	#So multiple listener could be run simultaneously
	rospy.init_node('listener',anonymous=True)
	rospy.Subscriber("joy",Joy,callback)
	
	#Spin() keeps python existing until the node is shut down
	rospy.sleep(0.01)
	rospy.spin()


#Direct Kinematics functions for a 6 DoF arm

#Array step needed for the dynamic movement
def vectorStep(Nstep,step):
     MaxStep=0
     for i in range (len(Nstep)):
         if(abs(Nstep[i]) >= MaxStep):
             MaxStep=abs(Nstep[i])
      
     Vector=[[],[],[],[],[],[]]        
     for j in range(len(Nstep)):
         for z in range(MaxStep):
             if(z < abs(Nstep[j])):
               Vector[j].append(step)
             else:
                  Vector[j].append(0)            
     return Vector

#Address calculation for each motor
def address(Nmotor):
	if(Nmotor==1 or Nmotor==2):
		address=0x24
	elif(Nmotor==5 or Nmotor==6):
		address=0x23
	else:
		address=0x25
	return address	

#PCF Byte calculation for each motor
def byteDirection(LR,ArmD):
    if(ArmD%2!=0):
        if(LR!=0):
            pulse='1'
            dire='0'
            if(LR>0):
                dire='1'
                v1='110001'+dire+'0'
                v2='110001'+dire+pulse
                v=[v1,v2]
                
            else:
                v1='110001'+dire+'0'
                v2='110001'+dire+pulse
                v=[v1,v2]

        else:
            pulse='0'
            dire='0'			
            v1='110001'+dire+'0'
            v2='110001'+dire+pulse
            v=[v1,v2]
        return v

    else: 
        if(LR!=0):
            pulse='1'
            dire='0'
            if(LR>0):
                dire='1'
                v1=dire+'100'+'0'+'110'
                v2=dire+'100'+pulse+'110'
                v=[v1,v2]
            else:
                v1=dire+'100'+'0'+'110'
                v2=dire+'100'+pulse+'110'
                v=[v1,v2]
        else:
            dire='0'
            pulse='0'
            v1=dire+'100'+'0'+'110'
            v2=dire+'100'+pulse+'110'
            v=[v1,v2]
        return v

#Enable modification for each motor
def enable(ArmD):
    enable='0'
    if(ArmD%2!=0):
        v1='11000'+enable+'0'+'0'
        v2='11000'+enable+'0'+'1'
        v=[v1,v2]
    else: 
        v1='0'+enable+'00'+'0'+'110'
        v2='0'+enable+'00'+'1'+'110'
        v=[v1,v2]
    return v


if __name__=='__main__':
	try:
		joy_listener()
	except rospy.ROSInterruptException:
		pass






