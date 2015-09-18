function [inouttimes,offset] = getInOutTimes(ptimesmatrix,offset)
%returns a matrix with column 1 being the times (ms) of stepping into or out of a
%function and column 2 wheter stepping in (1) or stepping out (0)
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================
    if nargin<2
        offset = ptimesmatrix(3,1);
    else
        offset=offset;
    end;
    times = ptimesmatrix(3:end-2,1:2);
    times = times';
    times = times(:);
    
    inouttimes = [(times-offset)/1e6 repmat([1;0],size(times,1)/2,1)];
end