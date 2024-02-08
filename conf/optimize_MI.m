function [omega, vel]=optimize_MI(k, p, vel_range, dt, N, wts, w, omega_range, lfn, bin_map, Z)

MIvals=zeros(numel(omega_range),numel(vel_range));

for oo=1:numel(omega_range)
    for dd=1:numel(vel_range)

        mmdl1=@(x) rt1d(x, vel_range(dd), omega_range(oo), dt);
        p(:,:,k+1) = pf_predict(p(:,:,k), mmdl1, diag(w));

        % Support for Z 
        Zsup=-pi:pi/6:pi;
        M=size(Zsup,2);

        pZ=zeros(1,M);
        for ll=1:M
            % p(Z_ll)=int_kk p(T_kk)*p(Z_ll|T_kk)
%             pZ(ll)=sum(wts.*lfn1(Zsup(:,ll),p(:,:,k+1), hfun, eta, r_visible));
            pZ(ll)=sum(wts.*lfn(Zsup(:,ll),Z,p(:,:,k+1),bin_map));
        end
        pZ=pZ(pZ~=0);
        
        % entropy of z
        Hz=-sum(pZ.*log(pZ));
        
        % work on this to speed up
        pZT=zeros(N,M);
        pZcT=zeros(N,M);
        for jj=1:N
            for ll=1:M
                % p(Z,T) = p(T)*p(Z|T)
%                 pZT(jj,ll)=wts(jj)*lfn1(Zsup(:,ll),p(:,jj,k+1), hfun, eta, r_visible);
                pZT(jj,ll)=wts(jj)*lfn(Zsup(:,ll),Z,p(:,jj,k+1),bin_map);
                % p(Z|T)
%                 pZcT(jj,ll)=lfn1(Zsup(:,ll),p(:,jj,k+1), hfun, eta, r_visible);
                pZcT(jj,ll)=lfn(Zsup(:,ll),Z,p(:,jj,k+1),bin_map);
            end
        end
    
    % entropy 
    pZcT1=pZcT(pZcT~=0 & pZT~=0);
    pZT1=pZT(pZcT~=0 & pZT~=0);
    Hzt=-sum(pZT1(:).*log(pZcT1(:)));
    MIvals(oo,dd)=(Hz-Hzt);
    end
end

[val, row]=max(MIvals);
[~, col] = max(val);
row = row(col);
omega=omega_range(row);
vel=vel_range(col);
end

