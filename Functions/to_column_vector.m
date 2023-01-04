% converts any vector into a column vector
function out = to_column_vector(in) 
    if isrow(in)
        out = in';
    else
        out = in;
    end
end