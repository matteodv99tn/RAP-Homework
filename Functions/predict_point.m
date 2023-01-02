
function pred_point = predict_point(line, theta)

    a   = line(1);
    b   = line(2);
    c   = line(3);
    den = a*cos(theta) + b*sin(theta);
    t   = - c / den;

    pred_point = t * [cos(theta); sin(theta)];

end