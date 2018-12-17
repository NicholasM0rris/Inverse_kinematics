
function moveXY(x,y)
% Inverse kinematics
loadlibrary('dynamixel','dynamixel.h');
libfunctions('dynamixel');

%The motors can move from 0 - 300 degrees, from 0 to 1024 goalPosition


DEFAULT_BAUDNUM = 1;
DEFAULT_PORTNUM = 5;
P_GOAL_POSITION = 30;
P_Moving = 46;
speed = 10;
res = calllib('dynamixel','dxl_initialize',DEFAULT_PORTNUM,DEFAULT_BAUDNUM);
calllib('dynamixel','dxl_write_word',1 ,speed,20);
calllib('dynamixel','dxl_write_word',4 ,speed,20);

a1 = 295;                           %a1, a2 are link lengths
a2 = 255+4;    
%q1, q2 are the angles/goal positions of the motors
if x<0
    
%Elbow down
q2 = acosd((x^2 + y^2 - a1^2 - a2^2)/(2*a1*a2));
q1 = atand(y/x) - atand((a2*sind(q2))/(a1+a2*cosd(q2)));

OG_q1 = q1;
OG_q2 = q2;

while q1<0

    q1 = -1*q1;

end
while q2<0
            
    q2 = -1*q2;
    
end

 if(q1>180)
        q1_over_180 = q1-180;
        q1 = 180 - q1_over_180;
end
if(q2>180)
        q2_over_180 = q2-180;
        q2 = 180 - q2_over_180;
end

Final_q1 = q1;
Final_q2 = q2;

q1_corrected = Final_q1 + 60;
q2_corrected = Final_q2 + 180;

%convert to motor angles
m1 = 3.41*q1_corrected;
m2 = 11.375*q2_corrected;
% Since m1 is inversed we swap the values

m1 = 1023 - m1;
m2 = 4095 - m2;


end

if x>=0
    
%Elbow up
q2u = -1*acosd((x^2 + y^2 - a1^2 - a2^2)/(2*a1*a2));
q1u = atand(y/x) + atand((a2*sind(q2u))/(a1+a2*cosd(q2u)));

OG_q1u = q1u;
OG_q2u = q2u;

while q1u<0

    q1u = -1*q1u;

end
while q2u<0
            
    q2u = -1*q2u;
    
end

 if(q1u>180)
        q1u_over_180 = q1u-180;
        q1u = 180 - q1u_over_180;
end
if(q2u>180)
        q2u_over_180 = q2u-180;
        q2u = 180 - q2u_over_180;
end

Final_q1u = q1u;
Final_q2u = q2u;

q1_corrected = Final_q1u + 60;
q2_corrected = Final_q2u + 180;

%convert to motor angles
m1 = 3.41*q1_corrected;
m2 = 11.375*q2_corrected;

m1 = 1023 - m1;
m2 = 4095 - m2;
end
    
    

 
%Move the arm
calllib('dynamixel','dxl_write_word',1,P_GOAL_POSITION,m1)  
calllib('dynamixel','dxl_write_word',4,P_GOAL_POSITION,m2)


%close the port
calllib('dynamixel','dxl_terminate');
end
