%% 逆运动学
% 输入：末端点位置、姿态、自由度、DH表 
% 输出：各关节的角度
function JointAngle = InverseKinematics(Pos, Euler_RPY, DOF, DH_Parameter)
    if(abs(Euler_RPY(2))>90)
        fprintf('*** note: pitch>90 degree, feed back會差一個pi *** \n')
    end
    DesiredCmd.Roll  = Euler_RPY(1) * pi/180;
    DesiredCmd.Pitch = Euler_RPY(2) * pi/180;
    DesiredCmd.Yaw   = Euler_RPY(3) * pi/180;
    DesiredCmd;
    %% Step 1. 建立姿态矩阵
    a  = DesiredCmd.Roll;
    b  = DesiredCmd.Pitch;
    c  = DesiredCmd.Yaw;
    RX = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];% RX
    RY = [cos(b) 0 sin(b); 0 1 0;-sin(b) 0 cos(b)]; % RY
    RZ = [cos(c) -sin(c) 0; sin(c) cos(c) 0;0 0 1]; % RZ
    OrienMat = RZ*RY*RX;  
    
    %% Step 2. 初始化相关参数
    DesiredCmd.P     = Pos;                          % desired position
    DesiredCmd.R     = OrienMat;                     % desired orientation
    DesiredCmd.Elbow = -1;                           % Elbow Up = -1, Elbow Down =  1
    DesiredCmd.Wrist = 1;                           % Wrist Up =  1, Wrist Down = -1
    DesiredCmd.L1    = DH_Parameter( 1, 3 );
    DesiredCmd.L2    = DH_Parameter( 2, 1 ); 
    DesiredCmd.L3    = DH_Parameter( 4, 3 );
    DesiredCmd.L4    = DH_Parameter( 6, 3 );         % d3 一般而言就是夾具長度
    DesiredCmd.Angle = zeros( 1, DOF );

    %% Step 3. 逆位置运动学
    DesiredCmd = InversePosition( DesiredCmd ); 

    %% Step 4. 逆姿态运动学              
    DesiredCmd = InverseOrientation( DesiredCmd, DH_Parameter );
    
    %% Step 5. 输出結果
    JointAngle = DesiredCmd.Angle;
end

%% 逆位置运动学
% 输入：末端点位置、末端点姿态、自由度、DH连杆表 
% 输出：第1~3轴的角度
function Cmd = InversePosition( Cmd )
    WristPos = Cmd.P - Cmd.L4 * Cmd.R( 1:3, 3 );  %Oc = [Xc ; Yc ; Zc] = [WristPos(1) WristPos(2) WristPos(3)]

    if( abs( WristPos(1) ) < 0.0001 && abs( WristPos(2) ) < 0.0001 )
        Cmd.Angle(1) = 0;
    else
        Cmd.Angle(1) = atan2( WristPos(2) , WristPos(1) );% Yc/Xc
    end
    xc_2 = WristPos(1)^2;
    yc_2 = WristPos(2)^2;

    s    = WristPos(3) - Cmd.L1;
    D    = acos(( Cmd.L2^2 + Cmd.L3^2 - xc_2 - yc_2 - s^2  )/( 2.0 * Cmd.L2 * Cmd.L3 ));
    disp(D);
    Cmd.Angle(3)= D - pi;
    disp(Cmd.Angle(3));

    Cmd.Angle(2) = atan2( s, sqrt( xc_2 + yc_2 ) ) - atan2(Cmd.L3*sin( Cmd.Angle(3) ), Cmd.L2 + Cmd.L3*cos( Cmd.Angle(3) ));
end

%% 逆姿态运动学
% 输入：末端点位置、末端点姿态、自由度、DH连杆表 
% 输出：第4~6轴的角度
function Cmd = InverseOrientation( Cmd, DH_Parameter )
    s1  = sin( Cmd.Angle(1) + DH_Parameter(1,4) );
    c1  = cos( Cmd.Angle(1) + DH_Parameter(1,4) );
    s23 = sin( Cmd.Angle(2) + DH_Parameter(2,4) + Cmd.Angle(3) + DH_Parameter(3,4) );
    c23 = cos( Cmd.Angle(2) + DH_Parameter(2,4) + Cmd.Angle(3) + DH_Parameter(3,4) );

    R0_3 = [    c1*c23    s1   c1*s23;
                s1*c23   -c1   s1*s23;
                   s23     0     -c23  ];
    R3_6 = R0_3' * Cmd.R;

    Cmd.Angle(5) = atan2( Cmd.Wrist * sqrt( 1 - R3_6(3,3)^2 ), R3_6(3,3) );

    if( abs( R3_6(3,3) ) > 0.9999 )

        Cmd.Angle(4) = 0;
        Cmd.Angle(6) = atan2( R3_6(2,1) , R3_6(1,1) );

    else
        if( Cmd.Wrist > 0 )
            Cmd.Angle(4) = atan2( R3_6(2,3) ,  R3_6(1,3) );
            Cmd.Angle(6) = atan2( R3_6(3,2) , -R3_6(3,1) );     

        else
            Cmd.Angle(4) = atan2( -R3_6(2,3) , -R3_6(1,3) );
            Cmd.Angle(6) = atan2( -R3_6(3,2) ,  R3_6(3,1) );  

        end
    end
end