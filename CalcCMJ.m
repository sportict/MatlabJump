function Jump = CalcCMJ(acc_z,vel_z,Fs)
    %% タイミングの抽出タイミング
    % foot-release:リニア加速度の鉛直成分の最小値
    % foot-contact:リニア加速度の鉛直成分が-0.5gよりも大きくなった時
    %              (リニア速度の極小値から遡って抽出)

    
    %% Jump構造体の中身
    % tc:接地時間
    % ta:滞空時間
    % height:跳躍高
    % peak_vel:[1,Foot_release,Foot_contact,nFr]のインデックス

    %% 初期設定
    g = 9.80665;
    nFr = length(acc_z);
    
    %% acc_zの最小値を取得（離地のタイミング）
    [min_acc,min_acc_index] = min(acc_z);
    fr = min_acc_index;
    
    %% vel_zのピーク値を取得（接地のタイミング）
    [min_vel,min_vel_index] = min(vel_z);

    %% min_velから遡って境界以下になったタイミングを取得
    i_frame = 1;
    peak_frame = min_vel_index;
    while true
        tmp_acc = acc_z(peak_frame - i_frame,1);
        if tmp_acc <= g
            break
        end
        i_frame = i_frame + 1;
    end
    fc = peak_frame - i_frame + 1;
    
    %% 加速度と速度からジャンプタイミングを抽出
    peak_IMU = [1,fr,fc,nFr];
    
    %% 接地時間の算出
    dt = 1 / Fs;
    tc = 0;
    ta = (fc - fr) * dt;
    height = 1 / 8 * g * ta ^ 2;
    dj_index = 0;
    
    %% 構造体に格納
    Jump.tc = tc;
    Jump.ta = ta;
    Jump.height = height;
    Jump.dj_index = dj_index;
    Jump.peak_IMU = peak_IMU;
end