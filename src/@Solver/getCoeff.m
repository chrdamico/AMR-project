

        function a=getCoeff(solv)
            
            omega=solv.omega;
            tstep=solv.tstep;
            tstep_prev=solv.tstep_prev;
            tstep_i=solv.tstep_i;
            tstep_e=solv.tstep_e;
            zmp_pos_des=solv.zmp_pos_des;
            zmp_pos_des_prev=solv.zmp_pos_des_prev;
            dcm_pos_des_eos=solv.dcm_pos_des_eos;
            dcm_pos_des_eos_prev=solv.dcm_pos_des_eos_prev;
            
            a0=-(tstep_i^3*zmp_pos_des - tstep_e^3*zmp_pos_des_prev - 3*tstep_e*tstep_i^2*zmp_pos_des + 3*tstep_e^2*tstep_i*zmp_pos_des_prev + dcm_pos_des_eos*tstep_i^3*exp(-omega*(tstep - tstep_e)) - dcm_pos_des_eos_prev*tstep_e^3*exp(omega*(tstep_i - tstep_prev)) - tstep_i^3*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + tstep_e^3*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) - 3*dcm_pos_des_eos*tstep_e*tstep_i^2*exp(-omega*(tstep - tstep_e)) + 3*dcm_pos_des_eos_prev*tstep_e^2*tstep_i*exp(omega*(tstep_i - tstep_prev)) + 3*tstep_e*tstep_i^2*zmp_pos_des*exp(-omega*(tstep - tstep_e)) - 3*tstep_e^2*tstep_i*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) - dcm_pos_des_eos*omega*tstep_e*tstep_i^3*exp(-omega*(tstep - tstep_e)) + dcm_pos_des_eos_prev*omega*tstep_e^3*tstep_i*exp(omega*(tstep_i - tstep_prev)) + omega*tstep_e*tstep_i^3*zmp_pos_des*exp(-omega*(tstep - tstep_e)) - omega*tstep_e^3*tstep_i*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) + dcm_pos_des_eos*omega*tstep_e^2*tstep_i^2*exp(-omega*(tstep - tstep_e)) - dcm_pos_des_eos_prev*omega*tstep_e^2*tstep_i^2*exp(omega*(tstep_i - tstep_prev)) - omega*tstep_e^2*tstep_i^2*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + omega*tstep_e^2*tstep_i^2*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)))/(tstep_e - tstep_i)^3;
            a1=-(6*tstep_e*tstep_i*zmp_pos_des - 6*tstep_e*tstep_i*zmp_pos_des_prev + 6*dcm_pos_des_eos*tstep_e*tstep_i*exp(-omega*(tstep - tstep_e)) - 6*dcm_pos_des_eos_prev*tstep_e*tstep_i*exp(omega*(tstep_i - tstep_prev)) - 6*tstep_e*tstep_i*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + 6*tstep_e*tstep_i*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) + dcm_pos_des_eos*omega*tstep_i^3*exp(-omega*(tstep - tstep_e)) - dcm_pos_des_eos_prev*omega*tstep_e^3*exp(omega*(tstep_i - tstep_prev)) - omega*tstep_i^3*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + omega*tstep_e^3*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) + dcm_pos_des_eos*omega*tstep_e*tstep_i^2*exp(-omega*(tstep - tstep_e)) - 2*dcm_pos_des_eos*omega*tstep_e^2*tstep_i*exp(-omega*(tstep - tstep_e)) + 2*dcm_pos_des_eos_prev*omega*tstep_e*tstep_i^2*exp(omega*(tstep_i - tstep_prev)) - dcm_pos_des_eos_prev*omega*tstep_e^2*tstep_i*exp(omega*(tstep_i - tstep_prev)) - omega*tstep_e*tstep_i^2*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + 2*omega*tstep_e^2*tstep_i*zmp_pos_des*exp(-omega*(tstep - tstep_e)) - 2*omega*tstep_e*tstep_i^2*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) + omega*tstep_e^2*tstep_i*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)))/(tstep_e - tstep_i)^3;
            a2=(3*tstep_e*zmp_pos_des + 3*tstep_i*zmp_pos_des - 3*tstep_e*zmp_pos_des_prev - 3*tstep_i*zmp_pos_des_prev + 3*dcm_pos_des_eos*tstep_e*exp(-omega*(tstep - tstep_e)) + 3*dcm_pos_des_eos*tstep_i*exp(-omega*(tstep - tstep_e)) - 3*dcm_pos_des_eos_prev*tstep_e*exp(omega*(tstep_i - tstep_prev)) - 3*dcm_pos_des_eos_prev*tstep_i*exp(omega*(tstep_i - tstep_prev)) - 3*tstep_e*zmp_pos_des*exp(-omega*(tstep - tstep_e)) - 3*tstep_i*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + 3*tstep_e*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) + 3*tstep_i*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) - dcm_pos_des_eos*omega*tstep_e^2*exp(-omega*(tstep - tstep_e)) + 2*dcm_pos_des_eos*omega*tstep_i^2*exp(-omega*(tstep - tstep_e)) - 2*dcm_pos_des_eos_prev*omega*tstep_e^2*exp(omega*(tstep_i - tstep_prev)) + dcm_pos_des_eos_prev*omega*tstep_i^2*exp(omega*(tstep_i - tstep_prev)) + omega*tstep_e^2*zmp_pos_des*exp(-omega*(tstep - tstep_e)) - 2*omega*tstep_i^2*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + 2*omega*tstep_e^2*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) - omega*tstep_i^2*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) + omega*tstep_e*tstep_i*zmp_pos_des*exp(-omega*(tstep - tstep_e)) - omega*tstep_e*tstep_i*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) - dcm_pos_des_eos*omega*tstep_e*tstep_i*exp(-omega*(tstep - tstep_e)) + dcm_pos_des_eos_prev*omega*tstep_e*tstep_i*exp(omega*(tstep_i - tstep_prev)))/(tstep_e - tstep_i)^3;
            a3=-(2*zmp_pos_des - 2*zmp_pos_des_prev + 2*dcm_pos_des_eos*exp(-omega*(tstep - tstep_e)) - 2*dcm_pos_des_eos_prev*exp(omega*(tstep_i - tstep_prev)) - 2*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + 2*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) - dcm_pos_des_eos*omega*tstep_e*exp(-omega*(tstep - tstep_e)) + dcm_pos_des_eos*omega*tstep_i*exp(-omega*(tstep - tstep_e)) - dcm_pos_des_eos_prev*omega*tstep_e*exp(omega*(tstep_i - tstep_prev)) + dcm_pos_des_eos_prev*omega*tstep_i*exp(omega*(tstep_i - tstep_prev)) + omega*tstep_e*zmp_pos_des*exp(-omega*(tstep - tstep_e)) - omega*tstep_i*zmp_pos_des*exp(-omega*(tstep - tstep_e)) + omega*tstep_e*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)) - omega*tstep_i*zmp_pos_des_prev*exp(omega*(tstep_i - tstep_prev)))/(tstep_e - tstep_i)^3;
            
            a=[a0,a1,a2,a3];
%             syms b0 b1 b2 b3 omega tstep tstep_prev tstep_i tstep_e zmp_pos_des zmp_pos_des_prev dcm_pos_des_eos dcm_pos_des_eos_prev
%              eq1= b3*tstep_i^3+b2*tstep_i^2+b1*tstep_i+b0==zmp_pos_des_prev + exp(omega*(tstep_i-tstep_prev))*(dcm_pos_des_eos_prev-zmp_pos_des_prev);
%              eq2= 3*b3*tstep_i^2+2*b2*tstep_i+b1==omega*exp(omega*(tstep_i-tstep_prev))*(dcm_pos_des_eos_prev-zmp_pos_des_prev);
%              eq3= b3*tstep_e^3+b2*tstep_e^2+b1*tstep_e+b0==zmp_pos_des+exp(omega*(tstep_e-tstep))*(dcm_pos_des_eos-zmp_pos_des);
%              eq4= 3*b3*tstep_e^2+2*b2*tstep_e+b1==omega*exp(omega*(tstep_e-tstep))*(dcm_pos_des_eos-zmp_pos_des);
%              
%              b=solve([eq1,eq2,eq3,eq4],[b0,b1,b2,b3]);
%              
%             b0=b.b0
%             b1=b.b1
%             b2=b.b2
%             b3=b.b3

        end