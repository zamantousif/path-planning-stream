clc;
clear;
close all;
%initialize

tf=100.00;
tstep = 0.10;
x0= 9;
y0= 5;
theta0= 0;
bx0= 5.00;
by0= 5.00;
vx0= 0.5;
vy0= 0.00;

bXtrain= [bx0];
bYtrain= [by0];
VXtrain= [vx0];
VYtrain= [vy0];
Xtrain= [-0.10];
zinit= [x0;y0;theta0];
figure;
radius_robot=0.3;
radius_obs= 1.5
for t= 0.00:0.10:tf
    
%obstacle prediction
    bx= bx0 + vx0*t;
    by= by0 + vy0*t;
    vx= vx0;
    vy= vy0;
    Xtrain =[Xtrain;t];
    bXtrain= [bXtrain;bx];
    bYtrain= [bYtrain;by];
    VXtrain= [VXtrain;vx];
    VYtrain= [VYtrain;vy];
    
    
    gprMdbx = fitrgp(Xtrain,bXtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdby = fitrgp(Xtrain,bYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdvx = fitrgp(Xtrain,VXtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdvy = fitrgp(Xtrain,VYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
      
    Xtrain_pred=[t;t+0.1;t+0.2;t+0.3;t+0.4;t+0.5;t+0.6;t+0.7,];
    
    [bxtestpred] = predict(gprMdbx,Xtrain_pred);
    [bytestpred] = predict(gprMdby,Xtrain_pred);
    [vxtestpred] = predict(gprMdvx,Xtrain_pred);
    [vytestpred] = predict(gprMdvy,Xtrain_pred);
    
    
    obstaclepred= [bxtestpred,bytestpred,vxtestpred,vytestpred];
    
%trajectory planning
    start = [x0;y0;theta0];
    traj= get_stream_trajectory(tstep,start,obstaclepred);

    x0= traj(1,2);
    y0= traj(2,2);
    theta0= traj(3,2);
    z=[x0;y0;theta0];
    zinit=[zinit,z];
%     figure('Name','Stream Flow');
%     plot(traj(1,:), traj(2,:),'r-');
%     hold on
    if x0*x0 + y0*y0 <= 1
        break;
    end
    
%     delete(vch);
%     delete(vch1);

    vch1 = viscircles([bx,by], radius_obs,'Color','r');
    plot(bxtestpred, bytestpred,'r-');
    vch = viscircles([x0,y0], radius_robot,'Color','g');
    plot(traj(1,:), traj(2,:),'g-');
    
    hold on;
    pause(0.01);
    delete(vch);
    delete(vch1);

end
    figure('Name','Stream Flow');
    plot(zinit(1,:), zinit(2,:),'r-');
    hold on
    plot(bXtrain, bYtrain,'r-');
    hold on

