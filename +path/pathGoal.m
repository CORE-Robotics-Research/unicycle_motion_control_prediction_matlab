% Example:
%   rng(0);
%   P = rand(3,2);
%   X = rand(3,2);
%   r = 0.4;
%   Y = path.pathGoal(P, X, r);
%   figure; hold on; box on; grid on; axis equal;
%   for k = 1:size(X,1)
%       patch('XData', X(k,1) + r*cos(linspace(0, 2*pi, 100)),...
%             'YData', X(k,2) + r*sin(linspace(0, 2*pi, 100)),...
%             'FaceColor', 'g', 'EdgeColor', 'g', 'FaceAlpha', 0.25);
%   end
%   plot(P(:,1), P(:,2), 'b-');
%   scatter(P(1,1), P(1,2), [], 'b', 'filled');
%   scatter(P(end,1), P(end,2), [], 'c', 'filled');
%   scatter(X(:,1), X(:,2), [], 'g', 'filled');
%   scatter(Y(:,1), Y(:,2), [], 'm');

function Y = pathGoal(P, X, R)

    if numel(R) == 1
        R = R*ones(size(X,1),1);
    end

    % Default path goal is the points itself
    switch size(P,1)
        case 0 % Empty navigation path
               % Do nothing
               Y = X;
        case 1 % Point target
               Y = X; 
               I = sum((X - P).^2, 2) < (R.^2);
               Y(I,:) = repmat(P, [sum(I), 1]);
        otherwise % Navigation path with at least two vertices
               Y = pathGoal_vectorized(P, X, R); 
    end

end

function Y = pathGoal_vectorized(P, X, R)



    n = size(P,1);
    
    Pbase = P(1:(n-1),:);
    Pdiff = diff(P,[],1);
    Pmag2 = sum(Pdiff.^2, 2);
    
    Y = X; 
    for k = 1:size(X,1)
        Xbase = X(k,:) - Pbase;
        Xmag2 = sum(Xbase.^2,2);
        XP = sum(Pdiff.*Xbase,2);
        W = XP./Pmag2;
        W = max(min(W,1), 0);
       
        D = (W.^2).*Pmag2 - 2*W.*XP + Xmag2;
        
        I = find(D < (R(k)^2), 1, 'last');
        if not(isempty(I))
            
            w1 = XP(I)./Pmag2(I);
            
            distAB = norm(w1*Pdiff(I,:) - Xbase(I,:));
            w2 = sqrt((R(k)^2 - distAB^2)/Pmag2(I));
            w = w1 + w2;
            w = max(min(w,1),0);
            Y(k,:) = w*Pdiff(I,:) + Pbase(I,:);
            
        end

    end
        
end


function Y = pathGoal_loop(P, X, r)
    Y = X;
    for k = 1: (size(P,1) - 1)
            A = P(k,:);
            B = P(k+1,:);
            dAB = norm(A -B); 
            if (dAB > 0)
               % Compute the closest point on (A,B) segment that is in safe zone
                w1 = (X - A)*(B-A)'/dAB^2;
                w1hat = max(min(w1, 1),0);
                distAB = norm(X - (1-w1hat)*A - w1hat*B);
                if (distAB < r)
                    w2 = sqrt(r^2 - distAB^2)/dAB;
                    w = w1 + w2;
                    w = max(min(w,1),0);
                    Y = (1-w) * A + w*B;
                end
            else
                if (norm(X - B) < r)
                    Y = B;
                end
            end
        end

end