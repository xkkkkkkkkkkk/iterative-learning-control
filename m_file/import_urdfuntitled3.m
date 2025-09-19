robot  = importrobot('kr10_r1100_2_urdf.urdf');
robot.DataFormat = 'column';
config = homeConfiguration(robot);
disp(robot);
show(robot);
showdetails(robot);

initialConfig = homeConfiguration(robot);