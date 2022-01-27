function Jump = CalcDropJump(acc_z,vel_z,Fs)
    %% タイミングの抽出タイミング
    % foot-contact:リニア加速度の鉛直成分が-0.5gよりも大きくなった時
    %              (リニア速度の極小値から遡って抽出)
    % foot-release:リニア加速度の鉛直成分の最小値
    
    %% Jump構造体の中身
    % tc:接地時間
    % ta:滞空時間
    % height:跳躍高
    % dj_index:DJ指数
    % peak_vel:[1,Foot_contact1,Foot_release,Foot_contact2,nFr]のインデックス

    %% 初期設定
    g = 9.80665;
    nFr = length(acc_z);
    
    %% vel_zのピーク値を取得（接地のタイミング）
    [pks_min,locs_min] = findpeaks(vel_z*-1,'MinPeakDistance',Fs*0.5);
    [tmp_min,tmp_index] = maxk(pks_min,2);
    tmp_min_vel = [locs_min,pks_min*-1];
    min_vel(1) = tmp_min_vel(min(tmp_index),1);
    min_vel(2) = tmp_min_vel(max(tmp_index),1);
    min_data(1) = tmp_min_vel(min(tmp_index),2);
    min_data(2) = tmp_min_vel(max(tmp_index),2);

    %% min_vel(1)から遡って境界以下になったタイミングを取得
    i_frame = 1;
    peak_frame = min_vel(1);
    while true
        tmp_acc = acc_z(peak_frame - i_frame,1);
        if tmp_acc <= g
            break
        end
        i_frame = i_frame + 1;
    end
    fc1 = peak_frame - i_frame + 1;

    %% min_vel(2)から遡って境界以下になったタイミングを取得
    i_frame = 1;
    peak_frame = min_vel(2);
    while true
        tmp_acc = acc_z(peak_frame - i_frame,1);
        if tmp_acc <= g
            break
        end
        i_frame = i_frame + 1;
    end
    fc2 = peak_frame - i_frame + 1;

    %% acc_zの最小値を取得（離地のタイミング）
    range_acc_z = acc_z(min_vel(1):min_vel(2),:);   % 速度の極小値の間の加速度の最小値を取得
    [pks_min,locs_min] = findpeaks(range_acc_z*-1);
    [min_data,min_index] = max(pks_min);
    fr = locs_min(min_index)+min_vel(1)-1;
    
    %% 加速度と速度からジャンプタイミングを抽出
    peak_IMU = [1,fc1,fr,fc2,nFr];
    
    %% 接地時間の算出
    dt = 1 / Fs;
    tc = (fr - fc1) * dt;
    ta = (fc2 - fr) * dt;
    height = 1 / 8 * g * ta ^ 2;
    dj_index = height / tc;
    
    %% 構造体に格納
    Jump.tc = tc;
    Jump.ta = ta;
    Jump.height = height;
    Jump.dj_index = dj_index;
    Jump.peak_IMU = peak_IMU;
end