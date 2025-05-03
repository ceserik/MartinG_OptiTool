classdef carClass
    %CAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        tracks       
        m          
        Izz        
        wheelbase  
        track      
        COGr        
        COG         
        l_f         
        l_r          
        COGz         
        wheelradius  
        steeringAngle_max 
        steeringAngle_maxRate 
        steeringAngleRear_max 
        %% Motor parameters
        gearRatio
        Mf_max       
        Mf_slewRate  
        Mf_jerk      
        Mr_max       
        Mr_slewRate  
        Yt_max       
        PwrMax      
        PwrMin      
        maxSpeed     
        AcpCapacity  
        BrakeBalance 
        PowertrainType    
        trqSpeedCharTrq   
        trqSpeedCharSpeed 
        losses 
        E 
        D 
        B 
        C
        Sus_t          
        tire_fxFactor 
        tire_fyFactor 
        ftire
        %% Aerodynamics parameters
        CL  
        CD 
        A 
        COP
        wingLength_f
        %% gepd parameters
        gepdToggle
        gepdMass
        gepdMaxPower
        gepdForceCoeff
        gepdCapacity = 1/3;                 % 0 to MaxPower percent in 3 seconds
        %% environment parameters
        g 
        airDensity 
        steeredAxle
        drivenAxle 
        discipline 
        gepdSlewRate

    end
    
    methods
        function obj = car()
            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

