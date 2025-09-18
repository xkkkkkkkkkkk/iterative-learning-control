%% 正运动学推导函式
% 输入：自由度、各关节的角度、DH连杆表 
% 输出：各关节的位置、各关节的坐标
function Info = ForwardKinematics( DOF, JointAngle, DH_Parameter )
    %% Step 1.初始化各关节位置向量与姿态矩阵
    Info.JointPos = [0 ;0 ;0];                              % 各关节的坐标位置矩阵 [ 3 x n ] , n = DOF
    Info.JointDir = [1 0 0; 0 1 0; 0 0 1];                  % 各关节的坐标向量矩阵 [ 3 x 3n ], n = DOF
    Info.T0_6     = [ 1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];  % 由正运动学求出的转移函数，T0_6 表示是从 Frame0 ~ Frame6

    %% Step 2.使用齐次转換矩阵求解正运动学
    for i = 1 : DOF
        A        = GenerateTransformationMatrices( JointAngle(i) , DH_Parameter(i,:) ); % 按照各轴的DH參数代入DH转移函数中
        Info.T0_6     = Info.T0_6 * A;                                                  % 將其作连乘的动作
        Info.JointPos = [ Info.JointPos Info.T0_6( 1:3, 4 ) ];                          % 储存关节的坐标位置
        Info.JointDir = [ Info.JointDir Info.T0_6( 1:3, 1:3 ) ];                        % 儲存关节的向量信息
    end
     % 按照Rotation 矩阵 求解 Pitch, Roll, Yaw

    %% Step 3.提取末端点位置向量与姿态矩阵
    Info.P = Info.T0_6( 1:3, 4 );
    Info.R = Info.T0_6( 1:3, 1:3 );
    R0_6   = Info.T0_6( 1:3, 1:3 );

    %% Step 4.分析末端点的姿态(Pitch Roll Yaw--z y x) 
    cal_err = 1*10^-8;
    if( abs(R0_6(3,1)-1) < cal_err)         
        Info.Yaw   = 0;
        Info.Pitch = -pi/2;
        Info.Roll  = atan2(-R0_6(1,2), -R0_6(1,3));


    elseif( abs(R0_6(3,1)+1) < cal_err )    
        Info.Yaw   = 0;
        Info.Pitch = pi/2;
        Info.Roll  = atan2(-R0_6(1,2), R0_6(1,3));

    else                                   
        Info.Roll  = atan2(R0_6(3,2), R0_6(3,3));
        Info.Pitch = asin(-R0_6(3,1));
        Info.Yaw   = atan2(R0_6(2,1), R0_6(1,1));
    end
    % 正規化角度(介於正負180度之間)
    Info.Roll  = normalize(Info.Roll);
    Info.Pitch = normalize(Info.Pitch);
    Info.Yaw   = normalize(Info.Yaw);


end

function ang = normalize(ang)
    while(ang > pi)
        ang = ang - 2*pi;
    end
    while(ang < -pi)
        ang = ang + 2*pi;
    end
end

%%  產生 转移函数
function A = GenerateTransformationMatrices( Theta, DH_Parameter )
%    Theta = 0;
    C_Theta = cos( Theta + DH_Parameter(4) );
    S_Theta = sin( Theta + DH_Parameter(4) );
    C_Alpha = cos( DH_Parameter(2) );
    S_Alpha = sin( DH_Parameter(2) ); 
    
    A = [   C_Theta   -1*S_Theta*C_Alpha        S_Theta*S_Alpha     DH_Parameter(1) * C_Theta; 
            S_Theta      C_Theta*C_Alpha     -1*C_Theta*S_Alpha     DH_Parameter(1) * S_Theta;
            0                    S_Alpha                C_Alpha                DH_Parameter(3);
            0                          0                      0                             1   ];
    
end