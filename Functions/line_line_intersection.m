% Given two lines in the form of [a b c] coefficients of a parametrization ax + by + c = 0, the 
% function returns the intersection point of the two lines. If the lines are parallel, the function
% returns the origin
function X = line_line_intersection(line1, line2)
    A = [line1(1:2); line2(1:2)];
    B = [line1(3); line2(3)];
    if abs(det(A)) <= 1e-6
        X = [0; 0];
    else
        X = - A\B;
    end
end