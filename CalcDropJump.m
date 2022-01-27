function Jump = CalcDropJump(acc_z,vel_z,Fs)
    %% �^�C�~���O�̒��o�^�C�~���O
    % foot-contact:���j�A�����x�̉���������-0.5g�����傫���Ȃ�����
    %              (���j�A���x�̋ɏ��l����k���Ē��o)
    % foot-release:���j�A�����x�̉��������̍ŏ��l
    
    %% Jump�\���̂̒��g
    % tc:�ڒn����
    % ta:�؋󎞊�
    % height:����
    % dj_index:DJ�w��
    % peak_vel:[1,Foot_contact1,Foot_release,Foot_contact2,nFr]�̃C���f�b�N�X

    %% �����ݒ�
    g = 9.80665;
    nFr = length(acc_z);
    
    %% vel_z�̃s�[�N�l���擾�i�ڒn�̃^�C�~���O�j
    [pks_min,locs_min] = findpeaks(vel_z*-1,'MinPeakDistance',Fs*0.5);
    [tmp_min,tmp_index] = maxk(pks_min,2);
    tmp_min_vel = [locs_min,pks_min*-1];
    min_vel(1) = tmp_min_vel(min(tmp_index),1);
    min_vel(2) = tmp_min_vel(max(tmp_index),1);
    min_data(1) = tmp_min_vel(min(tmp_index),2);
    min_data(2) = tmp_min_vel(max(tmp_index),2);

    %% min_vel(1)����k���ċ��E�ȉ��ɂȂ����^�C�~���O���擾
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

    %% min_vel(2)����k���ċ��E�ȉ��ɂȂ����^�C�~���O���擾
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

    %% acc_z�̍ŏ��l���擾�i���n�̃^�C�~���O�j
    range_acc_z = acc_z(min_vel(1):min_vel(2),:);   % ���x�̋ɏ��l�̊Ԃ̉����x�̍ŏ��l���擾
    [pks_min,locs_min] = findpeaks(range_acc_z*-1);
    [min_data,min_index] = max(pks_min);
    fr = locs_min(min_index)+min_vel(1)-1;
    
    %% �����x�Ƒ��x����W�����v�^�C�~���O�𒊏o
    peak_IMU = [1,fc1,fr,fc2,nFr];
    
    %% �ڒn���Ԃ̎Z�o
    dt = 1 / Fs;
    tc = (fr - fc1) * dt;
    ta = (fc2 - fr) * dt;
    height = 1 / 8 * g * ta ^ 2;
    dj_index = height / tc;
    
    %% �\���̂Ɋi�[
    Jump.tc = tc;
    Jump.ta = ta;
    Jump.height = height;
    Jump.dj_index = dj_index;
    Jump.peak_IMU = peak_IMU;
end