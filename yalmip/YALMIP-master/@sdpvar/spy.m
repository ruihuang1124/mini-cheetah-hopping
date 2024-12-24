function varargout = spy(X,variablesonly)
%SPY (overloaded)

if nargin < 2
    variablesonly = 0;
end
 if isa(X,'blkvar')
    X = sdpvar(X);
 end
    
if variablesonly 
    Z = X.basis(:,2:end); 
else
    Z = X.basis; 
end

Z = reshape(sum(abs(Z),2),X.dim(1),X.dim(2));
Z = Z~=0;
if nargout==0
    spy(Z)
else
    varargout{1}=Z;
end