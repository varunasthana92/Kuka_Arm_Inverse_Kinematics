%final solution for inverse kinematics
clear
%A1 ±170 °
%A2 ±120 °
%A3 ±170 °
%A4 ±120 °
%A5 ±170 °
%A6 ±120 °
%A7 ±175 °

%{
%Providing matrices

Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
        -sin(t1)             ,  cos(t1)             ,    0;
         sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

%Rwrist= Rb4T*Rd;
Rwrist = [ (2^(1/2)*cos(t2 - t4)*cos(t1))/2 - (2^(1/2)*sin(t2 - t4))/2, -cos(t2 - t4)*sin(t1),  (2^(1/2)*sin(t2 - t4))/2 + (2^(1/2)*cos(t2 - t4)*cos(t1))/2;
          -(2^(1/2)*sin(t1))/2                                        , -cos(t1)             , -(2^(1/2)*sin(t1))/2;
           (2^(1/2)*cos(t2 - t4))/2 + (2^(1/2)*sin(t2 - t4)*cos(t1))/2, -sin(t2 - t4)*sin(t1), (2^(1/2)*sin(t2 - t4)*cos(t1))/2 - (2^(1/2)*cos(t2 - t4))/2];

R47 = [ -sin(t5)*sin(t7) + cos(t5)*cos(t6)*cos(t7), -sin(t5)*cos(t7) - cos(t5)*cos(t6)*sin(t7),  cos(t5)*sin(t6);
         cos(t6)*sin(t5)*cos(t7) + cos(t5)*sin(t7),  cos(t5)*cos(t7) - cos(t6)*sin(t5)*sin(t7),  sin(t5)*sin(t6);
        -sin(t6)*cos(t7)                          ,  sin(t6)*sin(t7)                          ,  cos(t6)];

%To solve
%Rwist = R47;
%}

%vrep robot coordinate
% z= +7.8663e-02

%To solve
t1=0;
t2=0;
t3=0;
t4=0;
t5=0;
t6=0;
t7a=0;
t7b=0;

%robot parametrs
d1= 0.420;
d2= 0.400;
db= 0.360;

%defining end effector parametrs
dn= 0.2;
xd= 0.6;
yd= -0.3;
zd= 0.51;
Od=[ xd; yd; zd];

%End-Effector Orientation wrt world frame Rd
Rx=[ 1   0            0;
     0   cosd(180)   -sind(180);
     0   sind(180)    cosd(180)];
 
Ry=[  cosd(45)    0    sind(45);
      0           1    0;
     -sind(45)    0    cosd(45)];
 
Rd=Rx*Ry;

Ocd=Od-(dn*Rd*[0 0 1]');
xc= Ocd(1);
yc= Ocd(2);
zc= Ocd(3);
zcc= zc-db;
c4= ((xc^2) + (yc^2) + (zcc^2) - (d1^2) - (d2^2))/(2*d1*d2);
t1= atan2(-yc,-xc);
th1= t1*180/(22/7);
if (th1<-170) || (th1>170)
    disp('Out of Workspcae')
else
    %theta2 is solved by solving a quadratic equation with
    % A, B and C as coeeficients for cos(theta 2)

    A= (xc^2) + (yc^2) + (zcc^2);
    B= -(zcc*((xc^2) + (yc^2) + (zcc^2) + (d1^2) - (d2^2))/d1);
    C= zcc^2 - (d2^2)*(1 - (c4)^2);

    %using quadratic formula for theta2
    Del= (B^2 -4*A*C)^0.5;
    c2a= (-B+Del)/(2*A);
    c2b= (-B-Del)/(2*A);
    %checking if both values of quadratic satisfies the range of cos [-1,1]
    check_t2_a=0;
    check_t2_b=0;
    if(c2a<=1 && c2a>=-1)
        check_t2_a=1;
        temp_a= acos(c2a)*180/(22/7);
        if(temp_a<119 && temp_a>=-119)
            t2a=acos(c2a);
        else
            t2a=0;
        end
    end

    if(c2b<=1 && c2b>=-1)
        check_t2_b=1;
        temp_b= acos(c2b)*180/(22/7);
        if(temp_b<119 && temp_b>=-119)
            t2b=acos(c2b);
        else
            t2b=0;
        end
    end
    disp('-----------------------------------------------------------------------------------------------')
    disp('NOTE:')
    disp('Inverse Kinematics have been solved by locking the 3rd Joint. As the end effector is symmetric')
    disp('Hence any value for theta_7 will satisfy the problem, but is still calculated by utilizing')
    disp('Inverse Kinematics concepts and producing 2 values for theta_7')
    disp('-----------------------------------------------------------------------------------------------')
    disp(' ')
    disp('"x-Equal 1-k sol m" means the Case x has been satisfied for the axis constraints')
    disp('from axis-1 tll axis-k.')
    disp('m=1 means t6 has been considered same as calculated value of acos and m=2 means')
    disp('t6 is considered as -acos')
    disp(' ')
    output_order=['theta_1','  theta_2','  theta_3','  theta_4','  theta_5','  theta_6','  theta_7']
    theta=[0,0,0,0,0,0,0];
    sol=0;
    n=1;
    while(n<=9)
        switch n
            case 1
                temp4= acos(c4);
                temp2= t2a;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('1_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('1_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=2;
                        else
                            n=2;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('1_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=2;
                        else
                            n=2;
                        end
                    else
                        n=2;
                    end
                else
                    n=2;
                end

            case 2
                temp4= acos(c4);
                temp2= -t2a;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('2_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('2_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=3;
                        else
                            n=3;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('2_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=3;
                        else
                            n=3;
                        end
                    else
                        n=3;
                    end
                else
                    n=3;
                end

            case 3
                temp4= acos(c4);
                temp2= t2b;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('3_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('3_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=4;
                        else
                            n=4;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('3_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=4;
                        else
                            n=4;
                        end
                    else
                        n=4;
                    end
                else
                    n=4;
                end

            case 4
                temp4= acos(c4);
                temp2= -t2b;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('4_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('4_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=5;
                        else
                            n=5;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('4_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=5;
                        else
                            n=5;
                        end
                    else
                        n=5;
                    end
                else
                    n=5;
                end

            case 5
                temp4= -acos(c4);
                temp2= t2a;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('5_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('5_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=6;
                        else
                            n=6;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('5_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=6;
                        else
                            n=6;
                        end
                    else
                        n=6;
                    end
                else
                    n=6;
                end
               
            case 6
                temp4= -acos(c4);
                temp2= -t2a;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('6_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('6_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=7;
                        else
                            n=7;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('6_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=7;
                        else
                            n=7;
                        end
                    else
                        n=7;
                    end
                else
                    n=7;
                end

            case 7
                temp4= -acos(c4);
                temp2= t2b;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('7_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('7_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=8;
                        else
                            n=8;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('7_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=8;
                        else
                            n=8;
                        end
                    else
                        n=8;
                    end
                else
                    n=8;
                end

            case 8
                temp4= -acos(c4);
                temp2= -t2b;
                th2= temp2*180/(22/7);
                th4= temp4*180/(22/7);
                Tb4o = [ d1*cos(t1)*sin(temp2) - d2*cos(t1)*sin(temp4-temp2);
                         d1*sin(t1)*sin(temp2) - d2*sin(t1)*sin(temp4-temp2);
                         db + d2*cos(temp4-temp2) + d1*cos(temp2)];

                if (-0.01<Tb4o(1)-Ocd(1)) && (Tb4o(1)-Ocd(1)<0.01) && (-0.01<Tb4o(2)-Ocd(2)) && (Tb4o(2)-Ocd(2)<0.01) && (-0.01<Tb4o(3)-Ocd(3)) && (Tb4o(3)-Ocd(3)<0.01)
                    if (-120<=th2) && (th2<=120) && (-120<=th4) && (th4<=120)
                        t2= temp2;
                        t4= temp4;
                        disp('8_Equal 1-4')

                        Rb4T= [  cos(t2 - t4)*cos(t1),  cos(t2 - t4)*sin(t1),   -sin(t2 - t4);
                                -sin(t1)             ,  cos(t1)             ,    0;
                                 sin(t2 - t4)*cos(t1),  sin(t2 - t4)*sin(t1),    cos(t2 - t4)];

                        Rwrist= Rb4T*Rd;
                        temp5= atan2(Rwrist(2,3),Rwrist(1,3));       
                        temp6= acos(Rwrist(3,3));
                        temp7= atan2(Rwrist(3,2),-Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('8_Equal 1-7 sol1')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=9;
                        else
                            n=9;
                        end
                        temp5= atan2(-Rwrist(2,3),-Rwrist(1,3));
                        temp6= -acos(Rwrist(3,3));
                        temp7= atan2(-Rwrist(3,2),Rwrist(3,1));
                        
                        th5= temp5*180/(22/7);
                        th6= temp6*180/(22/7);
                        th7= temp7*180/(22/7);
                        if (-170<=th5) && (th5<=170) && (-120<=th6) && (th6<=120) && (-175<=th7) && (th7<=175)
                            t5= temp5;
                            t6= temp6;
                            t7= temp7;
                            disp('8_Equal 1-7 sol2')
                            theta(1)= t1*180/(22/7);
                            theta(2)= t2*180/(22/7);
                            theta(4)= t4*180/(22/7);
                            theta(5)= t5*180/(22/7);
                            theta(6)= t6*180/(22/7);
                            theta(7)= t7*180/(22/7);
                            %output_order
                            theta
                            sol=1;
                            n=9;
                        else
                            n=9;
                        end
                    else
                        n=9;
                    end
                else
                    n=9;
                end

            otherwise
                n=10;
        end
    end
    if (sol==0)
        disp('No solutions found, out of workspace');
    end
end