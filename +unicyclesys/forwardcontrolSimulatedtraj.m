function [T, X] = forwardcontrolSimulatedtraj(Pose, goal, gains, dx, dth)
   
    Pose = reshape(Pose, [], 3);
    goal = reshape(goal, [], 2);
    
    x = Pose(1:2);
    theta = Pose(3);   

    stopCond = dx;

    k = 1;
    T = [0];
    while(norm(x(k,:)-goal) > stopCond)

        [v, w] = unicyclesys.forwardcontrol([x(k,:), theta(k)], goal, gains);
        dt = min( abs(dx/v) , abs(dth/w));
        T = [T; T(end) + dt];

        x(k+1,:) = x(k,:) + dt*v*[cos(theta(k)) sin(theta(k))];
        theta(k+1,:)= theta(k,:)+ w*dt;

        k = k+1;
    end
    X = [x, theta];

end