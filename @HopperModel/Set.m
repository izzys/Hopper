function [HM] = Set( HM, varargin )
% Sets desired object properties
% Use as Set(HM,'m',3,'mh',10,_)

nParams = (nargin-1)/2;
if rem(nParams,1)~=0 || nargin<1
    error('Set failed: not enough inputs')
else
    for p = 1:nParams
        key = varargin{2*p-1};
        value = varargin{2*p};
        if ~isnumeric(value)
            error('Set failed: property value must be numeric');
        end
        
        switch key
            
            case 'm1' 
                HM.m1 = value;
            case 'm2' 
                HM.m2 = value;
            case 'l1'  
                HM.l1 = value;
            case 'l2' 
                HM.l2 = value;
            case 'l_leg'
                HM.l_leg = value;
            case 'damping'
                HM.damping = value;
            case 'spr_l0' 
                HM.spr_l0 = value;
            case 'spr_k' 
                HM.spr_k = value;
            case 'g' 
                HM.g = value;
                
            % %%%%%% % Render parameters % %%%%%% %

            case 'LineWidth'
                HM.LineWidth = value;
            otherwise
                error(['Set failed: ',key,' property not found']);
        end

    end
end

