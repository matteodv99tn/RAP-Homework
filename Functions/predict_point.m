
function pred_point = predict_point(line, theta)

    a   = line(1);
    b   = line(2);
    c   = line(3);
    den = a*cos(theta) + b*sin(theta);

    pred_point = -c .* [cos(theta); sin(theta)] / den;

end