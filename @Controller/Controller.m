classdef Controller < handle & matlab.mixin.Copyable


    
    properties
 
        
        stDim = 1;
        nEvents = 1;
        
        % Controller effort:
        dx = 0.1;
        del_dx;
        
        % Gains:
        Kcontroller = 0;
        
        % LQR parameters:
        lqr_Q;
        lqr_R;
        
        % Set keys 
        SetKeys = {'P_reset'};
        
        LiftOffOccured;
    end
    
    methods
        
        % %%%%%% % Class constructor % %%%%%% %
        function [Con] = Controller()
            Con;
        end
        
        function [Con] = Init(Con,Mod,Px,Bu)
            
            %Set controller parametrs:
            Q = Con.lqr_Q;
            R = Con.lqr_R;
            A = Px;
            B = Bu;
            
           % [K,~,e] = dlqr(A,B,Q,R);
            Con.Kcontroller = [-0.121115408481471 0.247845362816203 -0.009612460318476 -0.021535507531306];
   
        end
        
        function [Xdot] = Derivative(Con, t, X)
            Xdot = X;
        end
        
        function [value ,isterminal ,direction] = Events(Con, Xb)
           value=1; 
           isterminal=1;
           direction=1;
        end

        function u = Command(Con,X,ModelObj)
            u=0;
            if ~ModelObj.OnGround %&& Con.LiftOffOccured
                   
                u=Con.dx+Con.del_dx;
                if u<0
                u = 0;
                disp 'controller reached saturation'
                end
              %  disp(['Control value: '  num2str(u) ])
            end
 
        end
    end
end