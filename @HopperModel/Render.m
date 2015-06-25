 function [HM] = Render(HM,q)
            % Get all the positions required

            % Upper mass:         
            PosMass1=HM.GetPos(q,HM.x1);
            % Lower mass:
            PosMass2=HM.GetPos(q,HM.x2);
            % Leg:
            PosLeg=HM.GetPos(q,HM.leg);
            % Spring start:
            PosSpringTop=HM.GetPos(q,HM.spring_top);
            % Spring end:
            PosSpringBottom=HM.GetPos(q,HM.spring_bottom);
            
            if isempty(HM.RenderObj)
                
                
                % DrawLink : Position, Orientation, Z, Length, Width, Color
                
                HM.RenderObj.Mass1=DrawLink(HM,PosMass1, pi/2, 4, HM.l1, HM.w1, HM.Colors{1});
                hold on
                HM.RenderObj.Mass2=DrawLink(HM,PosMass2, pi/2, 1, HM.l2, HM.w2, HM.Colors{2});
                HM.RenderObj.Leg=DrawLink(HM,PosLeg, pi/2, 2, HM.l_leg, HM.w_leg, HM.Colors{3});
                HM.RenderObj.Spring=DrawSpring(HM,PosSpringBottom, PosSpringTop,3, []);
                axis equal
            else
                if ishandle(HM.RenderObj.Mass1.Geom)==0
                    % Graphic handles were destroyed and need to be renewed
                    HM.RenderObj=[];
                    HM=HM.Render(q);
                else
                    DrawLink(HM,PosMass1, pi/2, HM.RenderObj.Mass1);
                    DrawLink(HM,PosMass2, pi/2, HM.RenderObj.Mass2);
                    DrawLink(HM,PosLeg, pi/2, HM.RenderObj.Leg);
                    DrawSpring(HM,PosSpringBottom, PosSpringTop, 3, HM.RenderObj.Spring);
                end
            end            

       
        function [ res ] = DrawLink(HM, Pos, Or, z_or_Obj, Length, Width, Color)
            
            res = [];
            
            switch nargin
                case 4
                    Txy=makehgtform('translate',[Pos(1) Pos(2) 0]);
                    Rz=makehgtform('zrotate',-Or);
                    set(z_or_Obj.Trans,'Matrix',Txy*Rz);
                    res=1;
                case 7
                    res.Trans=hgtransform('Parent',gca);
                    Txy=makehgtform('translate',[Pos(1) Pos(2) z_or_Obj]);
                    Rz=makehgtform('zrotate',-Or);

                    coordX=[-Length/2, -Length/2, Length/2, Length/2];
                    coordY=[-Width/2, Width/2, Width/2, -Width/2];
                    coordZ=[z_or_Obj, z_or_Obj, z_or_Obj, z_or_Obj];

                    res.Geom=patch(coordX,coordY,coordZ,Color);
                    set(res.Geom,'EdgeColor',[0 0 0]);
                    set(res.Geom,'LineWidth',2*HM.LineWidth);

                    set(res.Geom,'Parent',res.Trans);
                    set(res.Trans,'Matrix',Txy*Rz);
            end
        end
        
        function [ res ] = DrawSpring(HM, PosS, PosE,Z, Obj)
            
            NumTurns=5;
            
            Center=(PosS+PosE)/2;
            Length=sqrt((PosE(1)-PosS(1))^2+(PosE(2)-PosS(2))^2);
            Orientation=atan2(PosE(2)-PosS(2),PosE(1)-PosS(1));

            coordX=zeros(1,NumTurns*2+2);
            coordY=zeros(1,NumTurns*2+2);

            Step=Length/NumTurns;

            coordX(1)=-Length/2;
            coordX(end)=Length/2;
            for i=1:NumTurns
                coordX(2*i)=coordX(1)+Step*(1/4+i-1);
                coordX(2*i+1)=coordX(1)+Step*(3/4+i-1);
                coordY(2*i)=HM.LinkWidth/2;
                coordY(2*i+1)=-HM.LinkWidth/2;
            end
            
            % Rotate spring
            Coords=[coordX; coordY];

            RotMatrix=[cos(Orientation) -sin(Orientation);
                       sin(Orientation)  cos(Orientation)];
                     
                 
            Coords=repmat(Center',1,NumTurns*2+2)+RotMatrix*Coords;

            
            if isempty(Obj)
                res=zeros(2,1);
                res(1)=plot3(Coords(1,:), Coords(2,:),Z*ones(1,length(Coords)), 'Color', [0, 0, 0], 'LineWidth', 2*HM.LineWidth);

            else
                set(Obj(1),'XData',Coords(1,:));
                set(Obj(1),'YData',Coords(2,:));
                res=1;
            end
        end
        
end