classdef HopperModel

    
    properties(Constant)

        % coordinates:
        x1 = 1;
        x2 = 2;
        leg = 3;
        spring_top = 4; 
        spring_bottom = 5;  

        nEvents = 3;
        stDim = 4;
        

    end
    
    properties

        nModCo;         % number of model coordinates (varies from stance to flight)
        
        m1=0.1;         % Lower link mass
        l1=0.1;         % Lower link length
        I1=0;           % Lower link inertia
        w1 = 0.125;     % Lower link width
        
        m2=0.5;         % Upper link mass
        l2=0.45;        % Upper link length
        I2=0;           % Upper link inertia
        w2 = 0.25;      % Upper link width

        m_leg=0;        % Leg mass
        l_leg=0.4;      % Leg length
        I_leg=0;        % Leg inertia
        w_leg = 0.1;    % Leg width
        
        spr_l0=0.5;     % Spring initial length
        spr_k=500;      % Spring constant
        damping=1;      % Spring damper
                
        % Ground interaction
        OnGround=0;     % set to 1 if foot is on the ground
                
        % Control input
        dx=0;
     
        % Render parameters
        RenderObj;        
        LineWidth=1;
        LinkWidth=0.04;
        Colors={[0.9,0.3,0.2],[0.7,0.7,0.8],[0.3, 0.3, 0.4],[0.5, 0.5, 1], [0, 0.4,0.1],[0.1,0,0.9]};
        %            red        light grey    dark grey         purple        green     dark blue     
                
        % Gravity:
        g=9.81;
        
        % choose linear equations (for debugging):
        LinearModel = 0;
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        function [HM] = HopperModel()
            HM;
        end
              
        function [xy] = GetPos(HM,q,which)
            switch which

                case HM.x1
                    xy= [0  q(1)];
                case HM.x2
                    xy= [0  q(2)];
                case HM.leg
                    xy = [ 0  ,  q(2)-HM.dx];
                case HM.spring_top
                    xy=[0  ,q(2)-HM.l_leg/2-HM.dx];
                case HM.spring_bottom
                    xy=[0,  q(1)+HM.l1/2];
            end

        end
        
        function [xy] = GetVel(HM,q,which)
            switch which

                case HM.x1
                    xy = q(3);
                case HM.x2
                    xy = q(4);

            end
        end
        
        function l_spring = GetSpringLength(HM,q)
            l_spring = norm( HM.GetPos(q,HM.spring_top)-HM.GetPos(q,HM.spring_bottom));
        end
        
        function l0_spring = SprL0(HM)
            if HM.OnGround
            l0_spring = HM.spr_l0+HM.l1+HM.l_leg/2+HM.dx;
            else
            l0_spring = HM.spr_l0+HM.l1/2+HM.l_leg/2+HM.dx;
            end
        end       
        
        function [qdot]=Derivative(HM,t,q) 
         
            % q = [ x1, x2, x1d, x2d ] 
            % qdot = [ x1d, x2d, x1dd, x2dd] 
           if ~HM.LinearModel
            
                if HM.OnGround
                    [qdot]=HM.Derivative2D(t,[q(2) ; q(4)]);
                     qdot = [0; qdot(1); 0 ; qdot(2)];
                else
                    [qdot]=HM.Derivative4D(t,q);
                end
                
           else
               
                if HM.OnGround
                    [A,B] = HM.GetABstance();
                    [qdot]= A*q+B;

                else
                    [A,B] = HM.GetABflight();
                    [qdot]= A*q+B;
                end
               
           end
                           
        end
        
     
        function [qdot]=Derivative2D(HM,t,q) %#ok<INUSL>
            % q = [ x2, x2d ] 
            [M,D,K,G,Q]=HM.GetMatrices2D();

            q_dot2=M\(-D*q(2)-K*q(1)-G);
              % q_dot2=pinv(M)*(-D*q(2)-K*q(1)-G);
               
            qdot=[q(2); q_dot2];
            
        end
        
        function [qdot]=Derivative4D(HM,t,q) %#ok<INUSL>
            
           % q = [ x1, x2, x1d, x2d ] 
           
           [M,D,K,G,Q] = HM.GetMatrices4D();
  
            q_dot2=M\(-D*q(3:4)-K*q(1:2)-G);
         %   q_dot2=pinv(M)*(-D*q(3:4)-K*q(1:2)-G);
            qdot=[ q(3:4) ; q_dot2];
        end
        
        function [M,D,K,G,Q]=GetMatrices2D(HM)

            M = [HM.m2];
                
            D = HM.damping;
                          
            K = HM.spr_k;

            G =  HM.m2*HM.g-HM.spr_k*HM.SprL0();
              
            Q = HM.spr_k;
        end
        
        function [M,D,K,G,Q]=GetMatrices4D(HM)
            

            M = [ HM.m1     0;
                    0    HM.m2];
                
            D = HM.damping*[  1  -1;
                             -1  1];
                          
            K = HM.spr_k*[ 1 -1;
                          -1  1];
                       
            G = [ HM.m1*HM.g+HM.spr_k*HM.SprL0();
                  HM.m2*HM.g-HM.spr_k*HM.SprL0()];
              
            Q = HM.spr_k*[-1 0 ;
                           0 1];
  
        end
        
        function [A,B] = GetABstance(HM)
            
          
         [M,D,K,G,Q]=HM.GetMatrices2D();
         
         MK = -inv(M)*K;
         MD = -inv(M)*D;
            
         A = [    0       0       0       0; 
                  0       0       0       1;
                  0       0       0       0;
                  0       MK      0       MD];
           
         MG = -inv(M)*G;

         B = [0 ; 0 ; 0 ; MG];
            
        end
        
       function [A,B] = GetABflight(HM)
           
         [M,D,K,G,Q]=HM.GetMatrices4D();
         
         MK = -inv(M)*K;
         MD = -inv(M)*D;
            
         A = [    0       0       1       0   ; 
                  0       0       0       1   ;
               MK(1,1) MK(1,2) MD(1,1) MD(1,2);
               MK(2,1) MK(2,2) MD(2,1) MD(2,2)];
           
         MG = -inv(M)*G;
         
         B = [0 ; 0 ; MG(1) ; MG(2)];
           
       end
        
        function [value ,isterminal ,direction] = Events(HM, q,FloorObj) 

            value=ones(HM.nEvents,1);
            isterminal=ones(HM.nEvents,1);
            direction=zeros(HM.nEvents,1);
            
            PosFoot=HM.GetPos(q,HM.x1);
        
            % Check for foot contact
            value(1)=PosFoot(2)-HM.l1/2-FloorObj.Surf(PosFoot(1));
            direction(1) = -1;
            if HM.OnGround
                isterminal(1)=0;
            end
            
            % Check for foot detachment
            value(2)=HM.spr_k*(HM.GetSpringLength(q)-HM.spr_l0)+HM.damping*q(4)-HM.m1*HM.g;
            direction(2) = 1;
            if ~HM.OnGround
                isterminal(2)=0;
            end
            
            % Check if will hop
            
            direction(3) = -1;
            if HM.OnGround
               value(3) = HM.will_hop_or_not(q);
               isterminal(3)=1;
               
            else
               value(3) = 1;
               isterminal(3)=0;
            end
        end
          
    end
    
end