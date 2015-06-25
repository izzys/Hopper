function mark_collor = MarkForFlag(Sim,Flag)
mark_collor = 'y';

switch Flag
    
    case   Sim.EndFlag_WillNotHop
        
        mark_collor = 'r';
        
    case   Sim.EndFlag_ICpenetrateGround
        
        mark_collor = 'y';
        disp 'IC penetrate Ground'

    case   Sim.EndFlag_WindowClosed
        
        mark_collor = 'y';
        disp 'Window Closed'

    case   Sim.EndFlag_StoppedByUser
        
        mark_collor = 'y';
        disp 'Stopped By User'

    case   Sim.EndFlag_EndOfTime
        
        mark_collor = 'r';
        disp 'shoud be another flag?'
   
    case   Sim.EndFlag_DoneNumberOfSteps
       mark_collor = 'y';
        disp 'Shouldnt happen'

    case   Sim.EndFlag_Converged
        
       mark_collor = 'g';
       
    case   Sim.EndFlag_TimeEndedBeforeConverge
        
               mark_collor = 'r'; 
            
end

