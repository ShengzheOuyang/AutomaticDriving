function Y = ode1(odefun,tspan,y0,varargin)
    if ~isnumeric(tspan)
      error('TSPAN should be a vector of integration steps.');
    end
    
    if ~isnumeric(y0)
      error('Y0 should be a vector of initial conditions.');
    end
    
    h = diff(tspan);
    if any(sign(h(1))*h <= 0)
      error('Entries of TSPAN are not in order.') 
    end  
    
    try
      f0 = feval(odefun,tspan(1),y0,varargin{:});
    catch
      msg = ['Unable to evaluate the ODEFUN at t0,y0. ',lasterr];
      error(msg);  
    end  
    
    y0 = y0(:);   % Make a column vector.
    if ~isequal(size(y0),size(f0))
      error('Inconsistent sizes of Y0 and f(t0,y0).');
    end  
    
    neq = length(y0);
    N = length(tspan);
    Y = zeros(neq,N);
    
    Y(:,1) = y0;
    for i = 1:N-1 
      Y(:,i+1) = Y(:,i) + h(i)*feval(odefun,tspan(i),Y(:,i),varargin{:});
    end
    Y = Y.';
