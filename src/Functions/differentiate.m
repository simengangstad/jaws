function ppdf = differentiate(ppf)
    % Spline Derivative
    ppdf = ppf;
    ppdf.order=ppdf.order-1;
    ppdf.coefs=ppdf.coefs(:,1:end-1).*(ppdf.order:-1:1);
end