function d = line_point_distance(line, p)

    a   = line(1);
    b   = line(2);
    c   = line(3);
    x   = p(1);
    y   = p(2);

    num = abs(a*x + b*y + c);
    den = sqrt(a^2 + b^2);

    d   = num / den;


end