% Example:
%   P = rand(5,2);
%   X = rand(6,2);
%   [D, C] = path.pathDist(P,X);
%   figure; hold on; box on; grid on; axis equal;
%   plot(P(:,1), P(:,2), 'k-', 'LineWidth', 2);
%   plot(X(:,1), X(:,2), 'ro');
%   plot(C(:,1), C(:,2), 'rx');
%   plot([X(:,1) C(:,1)]', [X(:,2) C(:,2)]', 'b');

function [D, C] = pathDist(P, X)
    
    n = size(P,1);
    m = size(X,1);
    d = size(P,2);
    
    if (n < 2)
        D = pdist2(X, P, 'euclidean');
        C = P;
        return;
    end
    
    [D, C] = pathDist_vectorized(P, X);
    
end

function [D, C] = pathDist_vectorized(P,X)

    n = size(P,1);
    m = size(X,1);
    d = size(P,2);
    
    Pbase = P(1:(n-1), :);
    Pdiff = P(2:n,:) - P(1:(n-1),:);
    
    
    Pbase2 = permute(repmat(Pbase, [1 1 m]), [3 1 2]);
    Pdiff2 = permute(repmat(Pdiff, [1 1 m]), [3 1 2]);
    Pmag2 = repmat(sum(Pdiff.^2, 2)', [m, 1]);
    Xbase2 = permute(repmat(X, [1 1 (n-1)]), [1 3 2]) - Pbase2;
    
    XP2 = sum(Xbase2.*Pdiff2,3);
    W2 = XP2./Pmag2;
    W2 = max(min(W2, 1), 0);
    
    D2 = (W2.^2).*Pmag2 - 2*W2.*XP2 + sum(Xbase2.^2,3);
    
    [D, I] = min(D2, [], 2);
    J = sub2ind([m, n-1], (1:m)', I);
    W = repmat(W2(J), [1 d]);
    
    C = W.*Pdiff(I,:)+ Pbase(I,:);
  
end
