classdef revoluteJointWithRotor
    %revoluteJointWithRotor 
    
    properties
        nv = 1
        nq = 1
        jointAxis
        rotorAxis
        gearRatio
        output_body = 1
        bodies = 2
    end
    
    methods        
        function [Xup, S, Sd, v] = kinematics(obj, Xtree, q, qdot, vp)
            [XJ_link , S_link ] = jcalc( ['R' obj.jointAxis], q);
            [XJ_rotor, S_rotor ] = jcalc( ['R' obj.rotorAxis], q*obj.gearRatio);
            
            flag = 1;
            if (isa(q,'sym'))
                flag = 0;
            end

            % Hotfix for symbolic REMOVE
            if flag
                Xup = Xtree;
            else
                Xupcopy = Xtree; % REMOVE
            end
            
            if flag
                Xup(1:6,:)   = XJ_link *Xup(1:6,:)  ;
                Xup(7:12 ,:) = XJ_rotor*Xup(7:12,:) ;
            else
                Xup1 = XJ_link *Xupcopy(1:6,:); % REMOVE
                Xup2 = XJ_rotor*Xupcopy(7:12,:) ; % REMOVE
                Xup = [Xup1; Xup2]; % REMOVE
            end

            S  = [S_link ; S_rotor*obj.gearRatio];
            if nargout > 2
                v  = Xup*vp + S*qdot;
                Sd = crm(v)*S;
            end 
        end
        
    end
end

