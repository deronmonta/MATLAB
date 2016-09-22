function animatedlineex() 

numpoints = 100000; 
    x = linspace(0,4*pi,numpoints); 
    y = sin(x); 
  
    figure 
    h = animatedline; 
    new = animatedline;
    axis([0,4*pi,-1,1]);
   
    
        
    for k = 1:50000
      addpoints(h,x(k),y(k)) 
        drawnow update
        h.Color = 'red';
          h.LineWidth = 3;
          h.LineStyle = '-.';
    end   
          for r = 50001: numpoints
           addpoints(new,x(r),y(r))
           drawnow update
          new.Color = 'green';
           new.LineWidth = 3;
           new.LineStyle = '-.';
          end
          
      
    end 