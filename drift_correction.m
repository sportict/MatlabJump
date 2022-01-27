function dotdat = drift_correction(dat,Fs,stationary)

time_int = 1 / Fs;

% Integrate acceleration to yield velocity
dotdat = zeros(size(dat));
for t = 2:length(dotdat)
    dotdat(t,:) = dotdat(t-1,:) + dat(t,:) * time_int;
    if(stationary(t) == 1)
        dotdat(t,:) = [0 0 0];     % force zero dotdatocity when foot stationary
    end
end


% Compute integral drift during non-stationary periods
dotdatDrift = zeros(size(dotdat));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = dotdat(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    dotdatDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Remove integral drift
dotdat = dotdat - dotdatDrift;