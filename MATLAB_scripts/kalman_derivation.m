syms x y v theta omega m Tl Tr  I Cd Ca dt r

state = [v*cos(theta)*dt + x + 1/2*(dt^2)*cos(theta)*(Tl+Tr-Cd*(v^2))/m;
         v*sin(theta)*dt + y + 1/2*(dt^2)*sin(theta)*(Tl+Tr-Cd*(v^2))/m;
         dt*(Tl+Tr-Cd*(v^2))/m + v;
         omega*dt + theta;
         dt*(Tr*r - Tl*r -Ca*(omega^2))/I + omega];

observation = [x;y; dt*(Tl+Tr-Cd*(v^2))/m;theta;omega];

G = jacobian(state,[x,y,v,theta,omega])
H = jacobian(observation,[x,y,v,theta,omega])

