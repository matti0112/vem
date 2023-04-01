import numpy as np
import math
import cairo
import random

# global variables
K=np.empty(shape=[0,0])
F=np.empty(shape=[0])
U=np.empty(shape=[0])
node_coordinates=[]
node_DoF=[]
connection_table=[]

# 4223

# Table of contents
Ev=[150, 170, 190, 210]
av=[3, 2.5, 2, 1.5]
dv=[50, 45, 40, 35]
bv=[1.3, 1.6, 1.9, 2.2]
Fv=[90, 130, 170, 210]
cv=[7, 6, 5, 4]

# Getting the input as the task code
key=input("Code: ")

# Data curry by task code
E=Ev[int(key[1])-1]*1000000000
a=av[int(key[1])-1]
d=dv[int(key[2])-1]/1000
b=bv[int(key[2])-1]
F1=Fv[int(key[3])-1]*1000
c=cv[int(key[3])-1]
A=math.pi*(1.15*d)**2/4-math.pi*d**2/4

# function: coordinates -> distance
def distance(a,b):
    return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

# function: coordinates -> angle
def angle(a,b):
    if math.atan2(b[1]-a[1],b[0]-a[0])<0:
        return math.atan2(b[1]-a[1],b[0]-a[0])+2*math.pi
    else:
        return math.atan2(b[1]-a[1],b[0]-a[0])

# function: x and y coordinates and DoF as bools -> updating global variables
def add_node(x,y,dx,dy):
    global K
    global F
    global U
    node_coordinates.append([x,y])
    node_DoF.append(dx)
    node_DoF.append(dy)
    K=np.pad(K,((0,2),(0,2)),constant_values=0)
    F=np.pad(F,(0,2),constant_values=0)
    U=np.pad(U,(0,2),constant_values=0)
    return True

# function: a and b are int #nums of the node -> update the stiffness matrix
def add_connection(a,b):
    connection_table.append([a,b])
    alf=angle(node_coordinates[a],node_coordinates[b])
    c=A*E/distance(node_coordinates[a],node_coordinates[b])
    
    K[a*2][a*2]+=c*math.cos(alf)**2
    K[a*2][a*2+1]+=c*math.cos(alf)*math.sin(alf)
    K[a*2+1][a*2]+=c*math.cos(alf)*math.sin(alf)
    K[a*2+1][a*2+1]+=c*math.sin(alf)**2
    
    K[a*2][b*2]+=c*-math.cos(alf)**2
    K[a*2][b*2+1]+=c*-math.cos(alf)*math.sin(alf)
    K[a*2+1][b*2]+=c*-math.cos(alf)*math.sin(alf)
    K[a*2+1][b*2+1]+=c*-math.sin(alf)**2
    
    K[b*2][a*2]+=c*-math.cos(alf)**2
    K[b*2][a*2+1]+=c*-math.cos(alf)*math.sin(alf)
    K[b*2+1][a*2]+=c*-math.cos(alf)*math.sin(alf)
    K[b*2+1][a*2+1]+=c*-math.sin(alf)**2
    
    K[b*2][b*2]+=c*math.cos(alf)**2
    K[b*2][b*2+1]+=c*math.cos(alf)*math.sin(alf)
    K[b*2+1][b*2]+=c*math.cos(alf)*math.sin(alf)
    K[b*2+1][b*2+1]+=c*math.sin(alf)**2
    return True

# function: int #num node, size in x and y directons -> updates force vector
def add_force(a,x,y):
    F[a*2]+=x
    F[a*2+1]+=y

# building the systems of coded case studies
if key[0]=='1':
    add_node(0,b,True,False)
    add_node(a,b,True,True)
    add_node(a+c,b,False,True)
    add_node(0,0,False,False)
    add_node(c,0,True,True)
    add_connection(0,1)
    add_connection(0,3)
    add_connection(1,2)
    add_connection(1,3)
    add_connection(1,4)
    add_connection(2,4)
    add_connection(3,4)
    add_force(1,0,-3*F1)
    add_force(2,0,F1)
    add_force(4,0,F1)
elif key[0]=='2':
    add_node(0,b,True,True)
    add_node(c,b,False,True)
    add_node(a,b/2,True,True)
    add_node(0,0,False,True)
    add_node(c,0,False,False)
    add_connection(0,1)
    add_connection(0,2)
    add_connection(0,3)
    add_connection(1,2)
    add_connection(2,3)
    add_connection(2,4)
    add_connection(3,4)
    add_force(1,0,-F1)
    add_force(2,-F1,0)
    add_force(3,0,3*F1)
elif key[0]=='3':
    add_node(a+c,b+b/2,True,False)
    add_node(0,b,False,True)
    add_node(a,b,True,True)
    add_node(a+c,b,True,True)
    add_node(a,0,False,False)
    add_connection(0,2)
    add_connection(0,3)
    add_connection(1,2)
    add_connection(1,4)
    add_connection(2,3)
    add_connection(2,4)
    add_connection(3,4)
    add_force(0,-3*F1,0)
    add_force(1,0,F1)
    add_force(2,0,-F1)
elif key[0]=='4':
    add_node(c,b+b/2,True,True)
    add_node(0,b,False,False)
    add_node(2*a+c,b,False,False)
    add_node(a,0,True,True)
    add_node(a+c,0,True,True)
    add_connection(0,1)
    add_connection(0,2)
    add_connection(0,3)
    add_connection(0,4)
    add_connection(1,3)
    add_connection(2,4)
    add_connection(3,4)
    add_force(0,0,-F1)
    add_force(3,0,6*F1)
    add_force(4,0,5*F1)
elif key[0]=='5':
    false=[0,1,2,3,4,5,6,7,8,9,10,19,20,29,30,39,40,49,50,59,60,69,70,79,80,89,90,91,92,93,94,95,96,97,98,99]
    co=0
    for i in range(10,0,-1):
        for j in range(10):
            if co in false:
                add_node(j,i,False,False)
            else:
                add_node(j,i,True,True)
            co+=1
    for i in range(len(node_coordinates)):
        for j in range(len(node_coordinates)):
            if distance(node_coordinates[i],node_coordinates[j])==1:
                add_connection(i,j)
    add_force(26,8000000,8000000)
elif key[0]=='6':
    xv=[]
    for i in range(100):
        x=random.randrange(1000,9000,1)
        y=random.randrange(1000,9000,1)
        while True:
            if x in xv:
                x=random.randrange(1000,9000,1)
            elif ((x-4500)**2+(y-4500)**2)>10000000 or ((x-4500)**2+(y-4500)**2)<4000000:
                x=random.randrange(1000,9000,1)
                y=random.randrange(1000,9000,1)
            else:
                break
        xv.append(x)
        add_node(x,y,random.choice([True,False]),random.choice([True,False]))
    for i in range(100):
        conv=[]
        for j in range(4):
            while True:
                p=random.randrange(0,100,1)
                if p!=i:
                    conv.append(p)
                else:
                    break
        add_connection(i,conv[0])
        add_connection(i,conv[1])
        add_connection(i,conv[2])
        add_connection(i,conv[3])
    for i in range(100):
        add_force(i,random.randrange(0,100000),random.randrange(0,100000))




# reduce the equation system
Ks=K
Fs=F
nodeD=np.array(node_DoF)
delete_counter=0
for i in range(len(node_DoF)):
    if node_DoF[i]==False:
        Ks=np.delete(Ks,i-delete_counter,0)
        Ks=np.delete(Ks,i-delete_counter,1)
        Fs=np.delete(Fs,i-delete_counter,0)
        nodeD=np.delete(nodeD,i-delete_counter)
        delete_counter+=1

# solving the reducated equation system
Us=np.linalg.solve(Ks,Fs)

# set up the whole equation system
c=0
U=[]
for i in range(len(node_DoF)):
    if node_DoF[i]==True:
        U.append(Us[c])
        c+=1
    else:
        U.append(0)

# solve the force vector
F=K.dot(U)

# calculate the relative change in length and the stresses of the beams
rel_length=[]
stress=[]
i=0
for item in connection_table:
    le=distance(node_coordinates[item[0]],node_coordinates[item[1]])
    alf=angle(node_coordinates[item[0]],node_coordinates[item[1]])
    rel_length.append(((U[item[1]*2]-U[item[0]*2])*math.cos(alf)+(U[item[1]*2+1]-U[item[0]*2+1])*math.sin(alf))/le)
    stress.append(E*rel_length[i]/1000000)
    i+=1

# function: list -> 0-1 num (correltated to min max value)
def make_relnum(list):
    out=[]
    mini=min(list)
    maxi=max(list)-mini
    for item in list:
        out.append((item-mini)/maxi)
    return out

# making the visual representation in vectorgraphics
with cairo.SVGSurface("struct.svg", 1000, 600) as surface:
    if key[0]=='6':
        
        color_correct=make_relnum(stress)
        context=cairo.Context(surface)
        context.set_line_width(10)
        context.set_source_rgb(0,0,0)
        context.paint()
        i=0
        for item in connection_table:
            cc=color_correct[i]
            if cc>0.48:
                context.set_source_rgb(1,1-cc*2,1-cc*2)
            else:
                context.set_source_rgb(1,1,1)
            context.move_to(node_coordinates[item[0]][0]+2000*U[item[0]*2], 10000-node_coordinates[item[0]][1]-2000*U[item[0]*2+1])
            context.line_to(node_coordinates[item[1]][0]+2000*U[item[1]*2], 10000-node_coordinates[item[1]][1]-2000*U[item[1]*2+1])
            context.stroke()
            i+=1
    
    else:
        #creating a cairo context object
        context = cairo.Context(surface)
        
        #draw constraints
        context.set_line_width(6)
        context.move_to(100+80*node_coordinates[1][0]+10, 500-80*node_coordinates[1][1]-30)        
        context.line_to(100+80*node_coordinates[1][0]-30, 500-80*node_coordinates[1][1]-30)
        context.line_to(100+80*node_coordinates[1][0]-30, 500-80*node_coordinates[1][1]+30)
        context.line_to(100+80*node_coordinates[1][0]+10, 500-80*node_coordinates[1][1]+30)
        context.stroke()
        context.move_to(100+80*node_coordinates[2][0]-10, 500-80*node_coordinates[2][1]-30)        
        context.line_to(100+80*node_coordinates[2][0]+30, 500-80*node_coordinates[2][1]-30)
        context.line_to(100+80*node_coordinates[2][0]+30, 500-80*node_coordinates[2][1]+30)
        context.line_to(100+80*node_coordinates[2][0]-10, 500-80*node_coordinates[2][1]+30)
        context.stroke()
        
        #draw forces
        context.set_source_rgba(1,0,0,0.4)
        context.set_line_width(8)
        
        context.move_to(100+80*node_coordinates[0][0], 500-80*node_coordinates[0][1])
        context.line_to(100+80*node_coordinates[0][0], 500-80*node_coordinates[0][1]+30)
        context.stroke()
        context.move_to(100+80*node_coordinates[0][0], 500-80*node_coordinates[0][1]+50)
        context.line_to(100+80*node_coordinates[0][0]-8, 500-80*node_coordinates[0][1]+50-24)
        context.line_to(100+80*node_coordinates[0][0]+8, 500-80*node_coordinates[0][1]+50-24)
        context.line_to(100+80*node_coordinates[0][0], 500-80*node_coordinates[0][1]+50)
        context.fill()
        

        context.move_to(100+80*node_coordinates[3][0], 500-80*node_coordinates[3][1])
        context.line_to(100+80*node_coordinates[3][0], 500-80*node_coordinates[3][1]-280)
        context.stroke()
        context.move_to(100+80*node_coordinates[3][0], 500-80*node_coordinates[3][1]-300)
        context.line_to(100+80*node_coordinates[3][0]-8, 500-80*node_coordinates[3][1]-300+24)
        context.line_to(100+80*node_coordinates[3][0]+8, 500-80*node_coordinates[3][1]-300+24)
        context.line_to(100+80*node_coordinates[3][0], 500-80*node_coordinates[3][1]-300)
        context.fill()
        
        context.move_to(100+80*node_coordinates[4][0], 500-80*node_coordinates[4][1])
        context.line_to(100+80*node_coordinates[4][0], 500-80*node_coordinates[4][1]-230)
        context.stroke()
        context.move_to(100+80*node_coordinates[4][0], 500-80*node_coordinates[4][1]-250)
        context.line_to(100+80*node_coordinates[4][0]-8, 500-80*node_coordinates[4][1]-250+24)
        context.line_to(100+80*node_coordinates[4][0]+8, 500-80*node_coordinates[4][1]-250+24)
        context.line_to(100+80*node_coordinates[4][0], 500-80*node_coordinates[4][1]-250)
        context.fill()
        
        # draw black lines
        context.set_source_rgba(0,0,0,0.2)
        context.set_line_width(20)
        for item in connection_table:
            #
            #
            context.move_to(100+80*node_coordinates[item[0]][0], 500-80*node_coordinates[item[0]][1])
            context.line_to(100+80*node_coordinates[item[1]][0], 500-80*node_coordinates[item[1]][1])
            context.stroke()
    
    
        # draw gray lines
        context.set_source_rgba(0.4,0.4,0.4,0.2)
        context.set_line_width(8)
        for item in connection_table:
            #+400*U[item[0]*2],-400*U[item[0]*2+1]
            #+400*U[item[1]*2],-400*U[item[1]*2+1]
            context.move_to(100+80*node_coordinates[item[0]][0], 500-80*node_coordinates[item[0]][1])
            context.line_to(100+80*node_coordinates[item[1]][0], 500-80*node_coordinates[item[1]][1])
            context.stroke()
    
        # draw black dots
        context.set_line_width(20)
        context.set_source_rgba(0,0,0,0.2)
        for i in range(int(len(node_DoF)/2)):
            #+400*U[i*2], -400*U[i*2+1]
            context.arc(100+80*node_coordinates[i][0], 500-80*node_coordinates[i][1], 10, 10, 20)
            context.stroke()
    
        # draw gray dots
        context.set_source_rgba(0.2,0.2,0.2,0.2)
        context.set_line_width(8)
        for i in range(int(len(node_DoF)/2)):
            #100+80*node_coordinates[i][0]+400*U[i*2], 900-80*node_coordinates[i][1]-400*U[i*2+1], 10, 10, 20
            context.arc(100+80*node_coordinates[i][0], 500-80*node_coordinates[i][1], 10, 10, 20)
            context.fill()
        
        
        
        
        
        
        
        # draw black lines
        context.set_source_rgb(0,0,0)
        context.set_line_width(20)
        for item in connection_table:
            #
            #
            context.move_to(100+80*node_coordinates[item[0]][0]+400*U[item[0]*2], 500-80*node_coordinates[item[0]][1]-400*U[item[0]*2+1])
            context.line_to(100+80*node_coordinates[item[1]][0]+400*U[item[1]*2], 500-80*node_coordinates[item[1]][1]-400*U[item[1]*2+1])
            context.stroke()
    
    
        # draw gray lines
        context.set_source_rgb(0.4,0.4,0.4)
        context.set_line_width(8)
        for item in connection_table:
            #+400*U[item[0]*2],-400*U[item[0]*2+1]
            #+400*U[item[1]*2],-400*U[item[1]*2+1]
            context.move_to(100+80*node_coordinates[item[0]][0]+400*U[item[0]*2], 500-80*node_coordinates[item[0]][1]-400*U[item[0]*2+1])
            context.line_to(100+80*node_coordinates[item[1]][0]+400*U[item[1]*2], 500-80*node_coordinates[item[1]][1]-400*U[item[1]*2+1])
            context.stroke()
    
        # draw black dots
        context.set_line_width(20)
        context.set_source_rgb(0,0,0)
        for i in range(int(len(node_DoF)/2)):
            #+400*U[i*2], -400*U[i*2+1]
            context.arc(100+80*node_coordinates[i][0]+400*U[i*2], 500-80*node_coordinates[i][1]-400*U[i*2+1], 10, 10, 20)
            context.stroke()
    
        # draw gray dots
        context.set_source_rgb(0.2,0.2,0.2)
        context.set_line_width(8)
        for i in range(int(len(node_DoF)/2)):
            #100+80*node_coordinates[i][0]+400*U[i*2], 900-80*node_coordinates[i][1]-400*U[i*2+1], 10, 10, 20
            #100+80*node_coordinates[i][0], 500-80*node_coordinates[i][1], 10, 10, 20
            context.arc(100+80*node_coordinates[i][0]+400*U[i*2], 900-80*node_coordinates[i][1]-400*U[i*2+1], 10, 10, 20)
            context.fill()
            
            
            
            
            
            
            
        # draw red dots
        #context.set_line_width(5)
        #context.set_source_rgb(1,0.2,0.2)
        #for i in range(int(len(node_DoF)/2)):
        #    context.arc(100+80*node_coordinates[i][0], 900-80*node_coordinates[i][1], 5, 10, 20)
        #    context.fill()

# printing message when file is saved
print("File Saved")
#for i in range(10):
#    print(round(K[7][i]/10000000,3))
#for i in range(10):
#    print(round(F[i]/1000,3))



#try