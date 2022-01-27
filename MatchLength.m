function [tt,acc,gyro,orien] = MatchLength(ta,ac,gy,or)

    dt = ta(2) - ta(1);
    nfa = length(ac);
    nfg = length(gy);
    nfo = length(or);
    mx = max([nfa,nfg,nfo]);
    acc = ac;
    gyro = gy;
    orien = or;
    if nfa ~= mx
        k = 1;
        diff = mx - nfa;
        for i = 1:diff
            acc(nfa+k,:) = ac(end,:);
            k = k + 1;
        end
    end
    if nfg ~= mx
        k = 1;
        diff = mx - nfg;
        for i = 1:diff
            gyro(nfg+k,:) = gyro(end,:);
            k = k + 1;
        end
    end
    if nfo ~= mx
        k = 1;
        diff = mx - nfo;
        for i = 1:diff
            orien(nfo+k,:) = or(end,:);
            k = k + 1;
        end
    end
    
    nf = length(acc);
    tt = 0:dt:(nf-1)*dt;
    tt = tt';

end