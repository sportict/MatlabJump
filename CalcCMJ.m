function Jump = CalcCMJ(acc_z,vel_z,Fs)
    %% �^�C�~���O�̒��o�^�C�~���O
    % foot-release:���j�A�����x�̉��������̍ŏ��l
    % foot-contact:���j�A�����x�̉���������-0.5g�����傫���Ȃ�����
    %              (���j�A���x�̋ɏ��l����k���Ē��o)

    
    %% Jump�\���̂̒��g
    % tc:�ڒn����
    % ta:�؋󎞊�
    % height:����
    % peak_vel:[1,Foot_release,Foot_contact,nFr]�̃C���f�b�N�X

    %% �����ݒ�
    g = 9.80665;
    nFr = length(acc_z);
    
    %% acc_z�̍ŏ��l���擾�i���n�̃^�C�~���O�j
    [min_acc,min_acc_index] = min(acc_z);
    fr = min_acc_index;
    
    %% vel_z�̃s�[�N�l���擾�i�ڒn�̃^�C�~���O�j
    [min_vel,min_vel_index] = min(vel_z);

    %% min_vel����k���ċ��E�ȉ��ɂȂ����^�C�~���O���擾
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
    
    %% �����x�Ƒ��x����W�����v�^�C�~���O�𒊏o
    peak_IMU = [1,fr,fc,nFr];
    
    %% �ڒn���Ԃ̎Z�o
    dt = 1 / Fs;
    tc = 0;
    ta = (fc - fr) * dt;
    height = 1 / 8 * g * ta ^ 2;
    dj_index = 0;
    
    %% �\���̂Ɋi�[
    Jump.tc = tc;
    Jump.ta = ta;
    Jump.height = height;
    Jump.dj_index = dj_index;
    Jump.peak_IMU = peak_IMU;
end