function d = point_point_distance(p1, p2)

    dx = p1(1) - p2(1);
    dy = p1(2) - p2(2);
    d = sqrt(dx.^2 + dy.^2);

end