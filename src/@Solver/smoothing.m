        function dcm=smoothing(solv)

            % exponential interpolation for single support and polynomial
            % interpolation for double support at a given state
            if solv.single_support

%                 'single'
                e=exp(solv.omega*(solv.t_curr-solv.tstep))*(solv.dcm_pos_des_eos-solv.zmp_pos_des);

                dcmPos=solv.zmp_pos_des+e;
                dcmVel=solv.omega*e;

                
            else
                % this function calculates the coefficients for the polynomial
                % interpolation. These are calculated from the double support phase 
                % boundary conditions
                a=getCoeff(solv);
%                 a=round(a*1e3)/1e3
%                 'double'
%                 solv.t_curr
%                 3*a(:,4)*solv.t_curr^2

                dcmPos=a(:,4)*solv.t_curr^3+a(:,3)*solv.t_curr^2+a(:,2)*solv.t_curr+a(:,1);
                dcmVel=3*a(:,4)*solv.t_curr^2+2*a(:,3)*solv.t_curr+a(:,2);
            end

            dcm=[dcmPos,dcmVel];
            dcm=double(dcm);

        end