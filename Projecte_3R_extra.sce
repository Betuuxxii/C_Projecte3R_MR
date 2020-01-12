//Definició de variables
L1 = 0.62
L2 = 0.57
g1 = 0.1
g2 = 0.2
g3 = 0.3

x = 0.9
y = -0.2
gamma = 0

mL1 = 4
mL2 = 3
mEE = 2


//----------------------------------TOTES LES FUNCIONS-----------------------------------//
// ----------------------Function inverse kinematics----------------------------//
function[theta1,theta2,theta3] = EndEffectorPose2joinPosition(x,y,gamma)
a12=L1
a23=L2
a34=g1

Rgamma = [cos(gamma), -sin(gamma);sin(gamma),cos(gamma)]
xyprima=[x;y]-Rgamma*[0;-g2]-Rgamma*[g3;0]

d = 2*a12*a23
f = (xyprima(1) - a34*cos(gamma))^2 + (xyprima(2) - a34*sin(gamma))^2 - (a12)^2 - (a23)^2
theta2 = acos(f/d)

if theta2 > %pi then
theta2 =2*%pi - theta2
end

a = a12 + a23*cos(theta2)
b = a23*sin(theta2)
e = xyprima(1) - a34*cos(gamma)
f = xyprima(2) - a34*sin(gamma)

matriutheta1 = inv([a,-b; b, a])*[e;f]
theta1 = atan(matriutheta1(2), matriutheta1(1))

if theta1 > %pi then
theta1 =2*%pi - theta1
end

theta3 = gamma -theta1 - theta2

endfunction


// ----------------------EndEffectorPosition2VelAngular----------------------------//
function[w1,w2,w3] = EndEffectorPosition2VelAngular(x,y,gamma, theta1,theta2,theta3)

J = [0 , L1*sin(theta1), L1*sin(theta1)+L2*sin(theta1+theta2); 0 , -L1*cos(theta1), -L1*cos(theta1)-L2*cos(theta1+theta2); 1,1,1]

velF=[0;-0.1;gamma]
velW = inv(J)*velF

w1 = velW(1)
w2 = velW(2)
w3 = velW(3)

endfunction



// ----------------------Plot3R----------------------------//
function [] = Plot3R(theta1,theta2,theta3)

// Càlcul de posicions de les joins
J1x = 0;
J1y = 0;
J2x = L1*cos(theta1);
J2y = L1*sin(theta1);
J3x = J2x + L2*(cos(theta1 + theta2));
J3y = J2y + L2*(sin(theta1 + theta2));
P_g1x = J3x + g1*(cos(theta3 + theta2 + theta1));
P_g1y = J3y + g1*(sin(theta3 + theta2 + theta1));
P_g2x = P_g1x;
P_g2y = P_g1y - g2;
P_g3x = P_g2x + g3;
P_g3y = P_g2y;

//Plot 3R arm
subplot(222)
title("3R motion", "fontsize",3)

a=get("current_axes") //eixos
a.data_bounds = [-0.2,-1;1,0] // minx, miny, maxx, maxy
a.axes_visible="on";

//Borrar anterior
a = gca();
delete(a.children);

//representació segments
xsegs([J1x,J2x],[J1y,J2y],1:1);
xsegs([J2x,J3x],[J2y,J3y],1:1);
xsegs([J3x,P_g1x],[J3y,P_g1y],1:1);
xsegs([P_g1x,P_g2x],[P_g1y,P_g2y],1:1);
xsegs([P_g2x,P_g3x],[P_g2y,P_g3y],1:1);

s = a.children
s.thickness=5

endfunction


// ----------------------Plot angular velocity----------------------------//
function[] = PlotJvsW(w1,w2,w3,k)
subplot(221)
title("Joint speeds vs time", "fontsize",3)
a=get("current_axes") //eixos
a.data_bounds = [0,-0.25;105,0.4] // minx, miny, maxx, maxy

plot2d(k,w1, 0)
e = gce();
point = e.children(1);
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=2;

plot2d(k,w2, 0);
e = gce();
point = e.children(1);
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=3;

plot2d(k,w3, 0);
e = gce();
point = e.children(1);
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=4;

hl=legend(['w1';'w2';'w3'])

endfunction


// ----------------------Plot wrench ----------------------------//
function[] = PlotJ1W(k,fJ1,mJ1)
subplot(223)
title("Wrench for J1","fontsize",3)

a=get("current_axes") //eixos
a.data_bounds = [0,0;110,150] // minx, miny, maxx, maxy

plot2d(k,fJ1,0)
e = gce()
point = e.children(1);
point.mark_mode="on";
point.mark_size = 1;
point.mark_foreground=2;

plot2d(k,mJ1,0)
e = gce()
point = e.children(1);
point.mark_mode="on";
point.mark_size = 1;
point.mark_foreground=3;

hl=legend(['F_J1','M_J1'])

endfunction


// ----------------------Càlcul wrench equivalent J1----------------------------//
function[weq] = WeqJ1(theta1,theta2,theta3)

j = [0,0,0;-1,-1,-1;-L1/2*cos(theta1),-L1*cos(theta1)-L2/2*cos(theta1+theta2),-L1*cos(theta1)-L2*cos(theta1+theta2)-(g1+g3/2)*cos(theta1+theta2+theta3)]

f = [9.81*mL1;9.81*mL2;9.81*mEE]

weq = -j*f

endfunction


//-----------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------//
//----------------------------MAIN---------------------------------------------------//

figure()
clf()

for k=1:100

//Inverse kinematics position
[theta1,theta2,theta3] = EndEffectorPose2joinPosition(x,y,gamma);
//Representació 3R
Plot3R(theta1,theta2,theta3);
//Cálcul vel angulars
[w1,w2,w3] = EndEffectorPosition2VelAngular(x,y,gamma, theta1,theta2,theta3);
//Plot Joint speeds vs time
PlotJvsW(w1,w2,w3,k);
//Afegir pes a l'EndEffector
if((k==20)|(k==40)|(k==60)|(k==80)|(k==100)) then
mEE = mEE+1;
end
//Càlcul wrench equivalent a J1
[weq] = WeqJ1(theta1,theta2,theta3);
PlotJ1W(k,weq(2),weq(3));

y= y-0.005;

//sleep(50);

end






