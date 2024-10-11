classdef optdata
    %OPTDATA Summary of this class goes here
    %   Detailed explanation goes here

    properties
        steering_f = esignal;
        alpha_f    = esignal;
        omega_f    = esignal;
        power_f    = esignal;
        Fy_f       = esignal;
        Fx_f       = esignal;
        Fz_f       = esignal;
        t;

        steering_r = esignal;
        alpha_r    = esignal;
        omega_r    = esignal;
        power_r    = esignal;
        Fy_r       = esignal;
        Fx_r       = esignal;
        Fz_r       = esignal;
        
        omega_fl    = esignal;
        power_fl    = esignal;
        Fy_fl       = esignal;
        Fx_fl       = esignal;
        Fz_fl       = esignal;
        FxC_fl      = esignal;
        FyC_fl      = esignal;

        omega_fr    = esignal;
        power_fr    = esignal;
        Fy_fr       = esignal;
        Fx_fr       = esignal;
        Fz_fr       = esignal;
        FxC_fr      = esignal;
        FyC_fr      = esignal;

        omega_rl    = esignal;
        power_rl    = esignal;
        Fy_rl       = esignal;
        Fx_rl       = esignal;
        Fz_rl       = esignal;
        FxC_rl      = esignal;
        FyC_rl      = esignal;

        omega_rr    = esignal;
        power_rr    = esignal;
        Fy_rr       = esignal;
        Fx_rr       = esignal;
        Fz_rr       = esignal;
        FxC_rr      = esignal;
        FyC_rr      = esignal;


        vx         = esignal;
        vy         = esignal;
        ax         = esignal;
        ay         = esignal;
        psi        = esignal;
        dpsi       = esignal;
        beta       = esignal;
        n          = esignal;
        F_trans    = esignal;
        Fz_aero    = esignal;
        Fdrag_aero = esignal;
        X          = esignal;
        Y          = esignal;

        alpha_fl = esignal;
        alpha_fr = esignal;
        alpha_rl = esignal;
        alpha_rr = esignal;
        e        = esignal; %empty esignal fro plotting before i think of something better
        YawTorque= esignal;
        
        rightFsum = esignal;
        leftFsum  = esignal;
        sideDiff  = esignal;

        YawMomentSteering = esignal;
        YawMomentTV       = esignal;
        PowerAcp          = esignal;
        PowerBrake        = esignal
        car;
        Fbrake            = esignal;


    end

    methods
        function obj = optdata(z_opt,u_opt,car)
            %OPTDATA Construct an instance of this class
            %Calcualtes all variables influencing the car and puts them
            %into structure
            set(0,'DefaultFigureWindowStyle','docked')
            
            %% car variables
            obj.car =car;
            obj.vx.name = "Longitudinal velocity [m/s]" ;
            obj.vy.name = "Lateral velocity [m/s]" ;
            obj.ax.name = "Longitudinal accceleration [m/s^2]" ;
            obj.ay.name = "Lateral accceleration [m/s^2]" ;
            obj.psi.name = "Heading [deg]" ;
            obj.dpsi.name = "Yaw rate [deg/s]" ;
            obj.beta.name = " Slip angle" ;
            obj.n.name = " distance from line [m]" ;
            obj.t.name = "Time [s]"; %time for states
            obj.X.name = "Position along X axis [m]";
            obj.Y.name = "Position along Y axis [m]";
            obj.rightFsum.name = "Forces right side [N]";
            obj.leftFsum.name = "Forces left side [N]";
            obj.sideDiff.name = "Side force difference [N]";
            obj.YawMomentSteering.name = "Yaw moment from steering";
            obj.YawMomentTV.name = " Yaw moment from TV";



            obj.vx.data = z_opt(:,1);
            obj.vy.data = z_opt(:,2);
            obj.psi.data =z_opt(:,3);
            obj.dpsi.data =z_opt(:,4);
            obj.n.data =z_opt(:,5);
            obj.F_trans = z_opt(:,7);
            
            obj.steering_r.data = u_opt(:,2);
            obj.t  = z_opt(:,6);
            obj.X.data = cumtrapz(obj.vx.data,obj.t);
            obj.Y.data = cumtrapz(obj.vy.data,obj.t);
           


            %% create front variables
            obj.steering_f.name = "Steering Angle front [deg]";
            obj.alpha_f.name    = "Slip angle front [deg]";
            obj.omega_f.name    = "Angular velocity front [rad/s]";
            obj.power_f.name    = "Power front [W]";
            obj.Fy_f.name       = "Lateral force front [N]" ;
            obj.Fx_f.name       = "Longitudinal force front [N]" ;
            obj.Fz_f.name       = "Normal force front [N]" ;

            obj.steering_f.data = u_opt(:,1);
            obj.Fx_f.data = u_opt(:,3);

            obj.omega_f.data =  (obj.vy.data(1:end-1) + car.l_f.*obj.dpsi.data(1:end-1)).*sin(obj.steering_f.data) + obj.vx.data(1:end-1).*cos(obj.steering_f.data) .* car.wheelradius;
            slipFrontRad = -atan2((obj.vy.data(1:end-1) + car.l_f.*obj.dpsi.data(1:end-1)).*cos(obj.steering_f.data) - obj.vx.data(1:end-1).*sin(obj.steering_f.data) ,...
                (obj.vy.data(1:end-1) + car.l_f.*obj.dpsi.data(1:end-1)).*sin(obj.steering_f.data) + obj.vx.data(1:end-1).*cos(obj.steering_f.data));
            
            obj.power_f.data = obj.omega_f.data .* obj.Fx_f.data;
            %obj.Fz_f.data = car.m/2 * car.g - obj.F_trans - obj.Fz_aero.data * (1-car.COP);
            %obj.Fy_f.data = car.ftire(slipFrontRad,obj.Fz_f.data(1:end-1), car);
        
            
            %% create rear variables
            obj.steering_r.name = "Steering Angle rear [deg]";
            obj.alpha_r.name    = "Slip angle rear [deg]";
            obj.omega_r.name    = "Angular velocity rear [rad/s]";
            obj.power_r.name    = "Power rear [W]";
            obj.Fy_r.name       = "Lateral force rear [N]" ;
            obj.Fx_r.name       = "Longitudinal force rear [N]" ;
            obj.Fz_r.name       = "Normal force rear [N]" ;

            if car.tracks == 2
                carVars = twinTransformations(z_opt,u_opt,car);
                %obj.ay.data = obj.dpsi.data(1:end-1) .* obj.vx.data(1:end-1);
                %% create front left variables
                obj.alpha_fl.name    = "Slip angle front left[deg]";
                obj.omega_fl.name    = "Angular velocity front keft[rad/s]";
                obj.power_fl.name    = "Power front left[W]";
                obj.Fy_fl.name       = "Lateral force front left[N]" ;
                obj.Fx_fl.name       = "Longitudinal force front left[N]" ;
                obj.Fz_fl.name       = "Normal force front left[N]" ;
                obj.FxC_fl.name      = "Long force Carrier front left[F]";
                obj.FyC_fl.name      = "Lateral force Carrier front left[F]";
                obj.Fx_fl.data       = carVars(:,1);
                obj.omega_fl.data    = carVars(:,37) / car.wheelradius;
                obj.power_fl.data    = carVars(:,49);
                obj.FxC_fl.data      = carVars(:,9);
                obj.FyC_fl.data      = carVars(:,13);
                obj.Fz_fl.data       = carVars(:,21);
                obj.Fy_fl.data       = carVars(:,5);



                %% create front right variables
                obj.alpha_fr.name    = "Slip angle front right[deg]";
                obj.omega_fr.name    = "Angular velocity front right[rad/s]";
                obj.power_fr.name    = "Power front right[W]";
                obj.Fy_fr.name       = "Lateral force front right[N]" ;
                obj.Fx_fr.name       = "Longitudinal force front right[N]" ;
                obj.Fz_fr.name       = "Normal force front right[N]" ;
                obj.FxC_fr.name      = "Long force Carrier front right [F]";
                obj.FyC_fr.name      = "Lateral force Carrier front righ[F]";
                obj.Fx_fr.data       = carVars(:,2);
                obj.omega_fr.data    = carVars(:,38) / car.wheelradius;
                obj.power_fr.data    = carVars(:,50);
                obj.FxC_fr.data      = carVars(:,10);
                obj.FyC_fr.data      = carVars(:,14);
                obj.Fz_fr.data       = carVars(:,22);
                obj.Fy_fr.data       = carVars(:,6);

                %% create rear left variables
                obj.alpha_rl.name    = "Slip angle rear left[deg]";
                obj.omega_rl.name    = "Angular velocity rear keft[rad/s]";
                obj.power_rl.name    = "Power rear left[W]";
                obj.Fy_rl.name       = "Lateral force rear left[N]" ;
                obj.Fx_rl.name       = "Longitudinal force rear left[N]" ;
                obj.Fz_rl.name       = "Normal force rear left[N]" ;
                obj.FxC_rl.name      = "Long force Carrier rear left [F]";
                obj.FyC_rl.name      = "Lateral force Carrier rear left [F]";
                obj.Fx_rl.data       = carVars(:,3);
                obj.omega_rl.data    = carVars(:,39) / car.wheelradius;
                obj.power_rl.data    = carVars(:,51);
                obj.FxC_rl.data      = carVars(:,11);
                obj.FyC_rl.data      = carVars(:,15);
                obj.Fz_rl.data       = carVars(:,23);
                obj.Fy_rl.data       = carVars(:,7);

                %% create rear right variables
                obj.alpha_rr.name    = "Slip angle rear right [deg]";
                obj.omega_rr.name    = "Angular velocity rear right [rad/s]";
                obj.power_rr.name    = "Power rear right [W]";
                obj.Fy_rr.name       = "Lateral force rear right [N]" ;
                obj.Fx_rr.name       = "Longitudinal force rear right [N]" ;
                obj.Fz_rr.name       = "Normal force rear right [N]" ;
                obj.FxC_rr.name      = "Long force Carrier rear right [F]";
                obj.FyC_rr.name      = "Lateral force Carrier rear right[F]";
                obj.Fx_rr.data       = carVars(:,4);
                obj.omega_rr.data    = carVars(:,40) / car.wheelradius;
                obj.power_rr.data    = carVars(:,52);
                obj.FxC_rr.data      = carVars(:,12);
                obj.FyC_rr.data      = carVars(:,16);
                obj.Fz_rr.data       = carVars(:,24);
                obj.Fy_rr.data       = carVars(:,8);

                obj.alpha_fl.name = "Slip angle front left [deg]";
                obj.alpha_fr.name = "Slip angle front right [deg]";
                obj.alpha_rl.name = "Slip angle rear left [deg]";
                obj.alpha_rr.name = "Slip angle rear right [deg]";


                %  [obj.alpha_fl.data, obj.alpha_fr.data ,obj.alpha_rl.data, obj.alpha_rr.data] = calWheels(z_opt, u_opt,car);
                obj.leftFsum.data  = obj.Fx_fl.data + obj.Fx_fl.data;
                obj.rightFsum.data = obj.Fx_fr.data + obj.Fx_rr.data;
                obj.sideDiff.data  = obj.rightFsum.data - obj.leftFsum.data;


                
                obj.alpha_fl.data = rad2deg(carVars(:,17));
                obj.alpha_fr.data = rad2deg(carVars(:,18));
                obj.alpha_rl.data = rad2deg(carVars(:,19));
                obj.alpha_rr.data = rad2deg(carVars(:,20));


                obj.Fdrag_aero.name = "Drag Force from aero [N]";
                obj.Fz_aero.name = "Aerodynamic downforce [N]";
                obj.YawTorque.name = "Yaw Torque [Nm]";
                obj.PowerAcp.name =" Power from ACP [W]";
                obj.PowerBrake.name =" Power from brakes [W}";


                obj.Fz_aero.data = -carVars(:,26);
                obj.Fdrag_aero.data = carVars(:,25);
                obj.YawTorque.data = carVars(:,46);
                obj.YawMomentSteering.data = carVars(:,48);
                obj.YawMomentTV.data = carVars(:,47);
                obj.PowerAcp.data = carVars(:,53);
                obj.PowerBrake.data = carVars(:,54);
                obj.ax.data =carVars(:,57); %diff(obj.vx.data)./diff(obj.t);
                obj.ay.data = carVars(:,58);%diff(obj.vy.data)./diff(obj.t);
                obj.Fbrake.data = carVars(:,59);
                obj.Fbrake.name = "Brake Force";

            end


            obj.steering_r.data = u_opt(:,2);
            obj.Fx_r.data = u_opt(:,4);

            obj.omega_r.data =  (obj.vy.data(1:end-1) - car.l_r.*obj.dpsi.data(1:end-1)).*sin(obj.steering_r.data) + obj.vx.data(1:end-1).*cos(obj.steering_r.data).* car.wheelradius;
            slipRearRad = -atan2((obj.vy.data(1:end-1) - car.l_r.*obj.dpsi.data(1:end-1)).*cos(obj.steering_r.data) - obj.vx.data(1:end-1).*sin(obj.steering_r.data) ,...
                (obj.vy.data(1:end-1) - car.l_r.*obj.dpsi.data(1:end-1)).*sin(obj.steering_r.data) + obj.vx.data(1:end-1).*cos(obj.steering_r.data));

            

            obj.power_r.data  = obj.omega_r.data .* obj.Fx_r.data;

%            obj.Fz_r.data = car.m/2 * car.g + obj.F_trans - obj.Fz_aero.data * (car.COP);
%            obj.Fy_r.data = car.ftire(slipRearRad,obj.Fz_r.data(1:end-1), car);
            


            %% cut last variables in states to make all vectors same length
            obj.vx.data = z_opt((1:end-1),1);
            obj.vy.data = z_opt((1:end-1),2);
            obj.psi.data = z_opt((1:end-1),3);
            obj.dpsi.data =z_opt((1:end-1),4);
            obj.n.data =z_opt((1:end-1),5);
            obj.F_trans = z_opt((1:end-1),7);
            %obj.Fz_aero.data = 0.5*car.airDensity*car.CL*car.A*obj.vx.data.^2;
            %% Convert radians to degrees 🤮
            obj.steering_r.data = rad2deg(obj.steering_r.data);
            obj.steering_f.data = rad2deg(obj.steering_f.data);
            obj.alpha_f.data = rad2deg(slipFrontRad);
            obj.alpha_r.data  = rad2deg(slipRearRad);
            obj.dpsi.data = rad2deg(obj.dpsi.data);
            obj.psi.data = mod(rad2deg(obj.psi.data),360); 
            %obj..name ="" ;

            %vx_opt    = z_opt(:,1); % longitudinal velocity of the car
            %vy_opt    = z_opt(:,2); % lateral velocity of the car
            %psi_opt   = z_opt(:,3); % orientation of the tcar
            %Dpsi_opt  = z_opt(:,4); % angular velocity of the car
            %n_opt     = z_opt(:,5); % perpendicular position of the car with respect to the track center line
            %t_opt     = z_opt(:,6); % time vector
            %Fz_opt    = z_opt(:,7); % Fz vector
            %Ft_opt    = t_opt(1:end-1); % time vector for the controls - this is just the time vector t_opt shorter by one element as we the controls are not applied at the end of the last control period (we have N values for each control input and N+1 values for each state)

            if (0==1)
            obj.plotControls;
            obj.plotStates;
            obj.plotSlip;
            obj.plotPwr;
            obj.plotLateral;
            obj.plotLongitudinal;
            obj.plotAero;
            obj.plotFz;
            end
        end

        function outputArg = method1(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        function  plotPwr(obj)
            %METHOD1 Summary of this method goes here
            %Plots power consmed by motors
            figure('Name','Power','NumberTitle','off')
            clf
            powerTotal = esignal;
            powerTotal.data = obj.power_fl.data + obj.power_fr.data + obj.power_rl.data + obj.power_rr.data;
            powerTotal.name = "Power Total [W]";
            eplot([obj.power_fr obj.power_fl   obj.power_rl obj.power_rr; 
                   powerTotal   obj.e          obj.e        obj.e ;
                   obj.PowerAcp obj.PowerBrake obj.e        obj.e 
                ],obj.t(1:end-1))
        end
        function plotControls(obj)
            figure('Name','Controls','NumberTitle','off')
            clf
            if obj.car.tracks == 2
                eplot([ ...
                    obj.steering_f obj.steering_r obj.e obj.e; ...
                    obj.Fx_fl obj.Fx_fr obj.Fx_rl obj.Fx_rr; ...
                    obj.Fbrake obj.e obj.e obj.e],obj.t(1:end-1))

            else
                obj.Fz_f.time = obj.t;
                obj.Fz_r.time = obj.t;
                eplot([ ...
                    obj.steering_f obj.steering_r;
                    obj.Fx_f       obj.Fx_r;
                    obj.Fz_f obj.Fz_r],obj.t(1:end-1))
            end
        end

        function plotSlip(obj)
            figure('Name','Slips','NumberTitle','off')
            clf
            eplot([obj.alpha_fl obj.alpha_fr obj.alpha_rl obj.alpha_rr],obj.t(1:end-1))

        end
        function plotStates(obj)
            figure('Name','States','NumberTitle','off')
            clf
            eplot([obj.vx obj.vy ; ...
                obj.ax obj.ay;...
                obj.dpsi obj.e ;...
                obj.alpha_r obj.alpha_f; ...
                ],obj.t(1:end-1))
        end
        function plotLateral(obj)
            figure('Name','Lateral variables','NumberTitle','off')
            matrix = [
                obj.YawMomentSteering obj.YawMomentTV obj.YawTorque obj.e
                obj.FyC_fl obj.FyC_fr obj.FyC_rl obj.FyC_rr
                obj.dpsi obj.e obj.e obj.e 
                obj.ay obj.e  obj.e obj.e 
            ];
            eplot(matrix,obj.t(1:end-1))

        end

        function plotLongitudinal(obj)
            figure('Name','Longitudinal variables','NumberTitle','off')
            matrix = [
                obj.FxC_fl obj.FxC_fr obj.FxC_rl obj.FxC_rr
                obj.ax obj.e obj.e obj.e 
                obj.vx obj.e  obj.e obj.e 
            ];
            eplot(matrix,obj.t(1:end-1))

        end
        function plotAero(obj)
            figure('Name','Aero','NumberTitle','off')
            matrix = [obj.Fdrag_aero obj.Fz_aero];
            eplot(matrix,obj.t(1:end-1))
        end
        function plotFz(obj)
            figure('Name','Normal Forces','NumberTitle','off')
            matrix = [obj.Fz_fl obj.Fz_fr obj.Fz_rl obj.Fz_rr];
            eplot(matrix,obj.t(1:end-1))
        end


        function plotTraj(obj)
            figure(44)
            clf
            scatter(obj.X.data,obj.Y.data)
        end

    end
end

