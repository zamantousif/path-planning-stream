
x0= 5.00;
y0= 10.00;
vx0= 0.00;
vy0= -1.00;

bXtrain= [x0];
bYtrain= [y0];
VXtrain= [vx0];
VYtrain= [vy0];
Xtrain= [-0.10];
for t = 0.00:0.10:3.00
    bx= x0 + vx0*t;
    by= y0 + vy0*t+ sin(t)*t;
    vx= vx0;
    vy= vy0+sin(t);
    Xtrain =[Xtrain;t];
    bXtrain= [bXtrain;bx];
    bYtrain= [bYtrain;by];
    VXtrain= [VXtrain;vx];
    VYtrain= [VYtrain;vy];
    
    gprMdby = fitrgp(Xtrain,bYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    ypred = resubPredict(gprMdby);
%     figure;
%     plot(Xtrain,bYtrain,'b.');
%     hold on;
%     plot(Xtrain,ypred,'r','LineWidth',1.5);
%     xlabel('x');
%     ylabel('y');
%     legend('Data','GPR predictions');
%     hold off
    Xtrain_pred=[t;t+0.1;t+0.2;t+0.3;t+0.4;t+0.5;t+0.6;t+0.7,];
    [ytestpred] = predict(gprMdby,Xtrain);
    [ytestpred2] = predict(gprMdby,Xtrain_pred);
%     figure;
%     plot(Xtrain,ytestpred,'b');
%     hold on;
%     plot(Xtrain_pred,ytestpred2,'g');
%     hold off
    gprMdvy = fitrgp(Xtrain,VYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    y_vypred = resubPredict(gprMdvy);
%     figure;
%     plot(Xtrain,VYtrain,'b.');
%     hold on;
%     plot(Xtrain,y_vypred,'r','LineWidth',1.5);
%     xlabel('x');
%     ylabel('y');
%     legend('Data','GPR predictions');
%     hold off
    [y_vytestpred,~,y_vytestci] = predict(gprMdvy,Xtrain);
    [y_vytestpred2,~,y_vytestci2] = predict(gprMdvy,Xtrain_pred);
    
    end
    figure;
    plot(Xtrain,y_vytestpred,'b');
    hold on;
    plot(Xtrain_pred,y_vytestpred2,'g');
    xlabel('x');
    ylabel('y');
    legend('Data','GPR predictions')
    hold off;    
    
    
    