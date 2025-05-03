load('15ms3.mat')

realWheelAngle = stw*0.285-1.5245;
realAy         = ay;
hold off
figure(11)
scatter(realAy,realWheelAngle,'filled')
out =sim("twintrackSimulinkRampSteer.slx");
hold on
scatter(out.simout(:,1),out.simout(:,2)*180/pi,'filled')
grid on
hold off
legend("Measured data","Vehicle model")
xlabel("Lateral acceleration [$m/s^2$]")
ylabel("Steering angle[deg]")
xlim([-20 0])