function [S]=k_seg_soft(X,k_max,alpha,lambda)
%
% Author: J.J. Verbeek   8/2/2001
% this software implements the soft-version of the k-segments algorithm
% to find principal curves.
%
% Please visit http://carol.wins.uva.nl/~jverbeek/ for more information
% and the accompanying papers and implementation of the hard-version.
%
% This software may be used for non-commercial use, if the original author
% and corresponding url are mentioned.
%
% [edges,vertices,of,y]=k_seg_soft(X,k_max,alpha,lambda,INT_PLOT)
%
% X        : n by d matrix of n d-dimensional data points
% k_max    : maximal number of segments
% alpha    : 2*(d-1)*sigma^2 -> the smoothing parameter
% lambda   : the angle-penalty weighter
% INT_PLOT : plot segments when updating if 1
%
% edges    : connectivity matrix 0: vertices not connected
%                                1: vertices connected by PL construction
%                                2: vertices are endpoints of same segment
% vertices : contains endpoints of segments
% of       : contains evaluations of objective function (third column),
%                     mean squared distance to curve (first column) and
%                     log of length of curve (second column)
% y        : contains data projected to curve (columns 2:end) plus projection indices (first column)
%

% tic
DO_OF_PLOTS=0;  % generate plots of objective function if 1
% data_opt='b.';  % plotting options for datapoints

% fprintf('computing square distances\n');
pwdists=sqdist(X',X');
of=zeros(k_max, 3);
f=1.5;f_ins=f;
lines=zeros(size(X,2),2,k_max);

k=1; %start with one line
[v,l]=eig(cov(X)); v = real(v);l = real(l);
[l,m]=max(diag(l)); 
if m == 1
    vTmp = v;
    vTmp(1,1) = v(2,1);
    vTmp(2,2) = v(1,2);
    vTmp(2,1) = -v(1,1);
    vTmp(1,2) = -v(2,2);
    v = vTmp;
    m = 2;
end
cent=mean(X);
lines(:,:,1)=[cent' - v(:,m)* f*sqrt(l) cent' + v(:,m)* f*sqrt(l)];

%compute projection distances
dists=inf(size(X,1),k_max);
for i=1:k
    [d p t]=seg_dist(lines(:,1,i),lines(:,2,i),X'); 
    dists(:,i)=d;
end
vr=ones(size(X,1),k_max);
dr=zeros(k_max,1);

[cost,edgesTmp]=construct_hp(lines(:,:,1:k),lambda); % construct
edgesTmp=optim_hp(edgesTmp,cost);            % 2-opt optimize
edges1 = edgesTmp(1:(end-1), 1:(end-1));
% edges(end,:)=[];
% edges(:,end)=[];

% hold off;
% if size(X,2)==2
%     plot(X(:,1),X(:,2),data_opt);
% elseif size(X,2)==3
%     plot3(X(:,1),X(:,2),X(:,3),data_opt);
% end
% hold on;
% plot_pad(edges,lines(:,:,1:k));   drawnow;

% fprintf('mapping points to arc-length\n');
vertices1=reshape(lines(:,:,1:k),size(X,2),2*k);
[y,sqd]=map_to_arcl(edges1,vertices1,X);
of(1, :) = [mean(sqd) log(max(y(:,1))) mean(sqd)+2*alpha*log(max(y(:,1)))];
y_best=y;
sigma=mean(sqd);

mu = zeros(k_max, 2);
pl = zeros(k_max, 1);
a = zeros(k_max, 1);

mu(1, :) = mean(X);
pl(1) = [1];
a(1) = [norm(lines(:,1,1)-lines(:,2,1))];

old_loglik=-inf;

while k<k_max
%     distsTmp = dists(:, 1:k);
    d=min(dists,[],2);
%     fprintf('inserting line nr. %d\n', k+1);
    dcp=max(repmat(d,1,size(X,1))-pwdists,0); %dists to vr points
    vr_size = sum(min(dcp,eps)./eps,1);
    vr_ns=min(max(vr_size,2),3)-2; % indicate vr_size>2
    delta=vr_ns.*sum(dcp,1);
    [t,i]=max(delta);
    indeces=find((dcp(:,i))>0);
    if length(indeces)<3
        k=k_max;
%         fprintf('allocation not possible anymore!\n');
        break
    end
    XS=X(indeces,:);
    k=k+1;
    cent=mean(XS);
    [v,l]=eig(cov(XS-repmat(cent,size(XS,1),1))); 
    v = real(v); l = real(l);
    [l,m]=max(diag(l));
    if m == 1
        vTmp = v;
        vTmp(1,1) = v(2,1);
        vTmp(2,2) = v(1,2);
        vTmp(2,1) = -v(1,1);
        vTmp(1,2) = -v(2,2);
        v = vTmp;
        m = 2;
    end
    lines(:,:,k)=[cent'-v(:,m)*f_ins*sqrt(l) cent'+v(:,m)*f*sqrt(l)];

%     plot([lines(1,1,k);lines(1,2,k)],[lines(2,1,k);lines(2,2,k)],'r');
%     drawnow;
    mu(k, :) = cent;
    a(k, :) = 2*f_ins*sqrt(l);
    pl=pl*(k-1)/k;
    pl(k, :) = 1/k;

    change=1;
    [d p t]=seg_dist(lines(:,1,k),lines(:,2,k),X'); dists(:,k)=d;
    [d p t]=seg_dist(lines(:,1,k),lines(:,2,k),XS');
    old_sigma=sigma;
    sigma=(k-1)*old_sigma/k+mean(d)/(2*k);

    px = zeros(size(X,1),k);
    for i=1:k % compute probabilities
        px(:,i) = exp(-dists(:,i)/(2*sigma))/(a(i)*2*pi*sigma);
    end

    %fprintf('refitting lines');
    change=1;  %indicates whether voronoi regions have changed
    iter=1;
    temp_sig=zeros(k,1);

    pix=zeros(size(X,1),k);
    for i=1:k % compute probabilities
        pix(:,i) = exp(-dists(:,i)/(2*sigma))/(a(i)*2*pi*sigma);
        pix(:,i) = pix(:,i)*pl(i); % pix contains now p(i,x) for pairs x,i
    end
    loglik = sum(log(sum(pix,2)));
    if loglik<old_loglik
%         fprintf('insertion did NOT improve Log-Likelihood!!\n');
    end

    while ((change>exp(-19)) && (iter<1150))
        %fprintf('.');
        old_lines=lines;
        pix=zeros(size(X,1),k);
        for i=1:k % compute probabilities
            pix(:,i) = exp(-dists(:,i)/(2*sigma))/(a(i)*2*pi*sigma);
            pix(:,i) = pix(:,i)*pl(i);
            % pix contains now p(i,x) for pairs x,i
        end

        loglik = sum(log(sum(pix,2)));

        pix = pix./repmat(sum(pix,2),1,k);  % pix contains p(i|x) for pairs x,i
        for i=1:k % compute new parameters
            weights = pix(:,i)/sum(pix(:,i));
            mu(i,:) = sum(X.*repmat(weights,1,size(X,2)));
            td = (X-repmat(mu(i,:),size(X,1),1));
            wtd =  td.*repmat(weights,1,size(X,2));
            [v,l]=eig(cov(wtd));
            v = real(v); l = real(l);
            [l,m]=max(diag(l));
            if m == 1
                vTmp = v;
                vTmp(1,1) = v(2,1);
                vTmp(2,2) = v(1,2);
                vTmp(2,1) = -v(1,1);
                vTmp(1,2) = -v(2,2);
                v = vTmp;
                m = 2;
            end
            var_on_pca = sum(((td*v(:,m)).^2).*weights);
            lines(:,:,i)=[mu(i,:)'-v(:,m)*(f*sqrt(var_on_pca)-sigma), mu(i,:)'+v(:,m)*(f*sqrt(var_on_pca)-sigma)];
            a(i)= 2*f*sqrt(var_on_pca);
            [d p t]=seg_dist(lines(:,1,i),lines(:,2,i),X'); 
            dists(:,i)=d;
            pl(i)=sum(pix(:,i))/size(X,1);
            temp_sig(i)=sum(d.*pix(:,i));
        end
        sigma = sum(temp_sig)/(2*size(X,1));

        pl=max(pl,10/size(X,1));
        pl=pl/sum(pl);
        sigma=max(sigma,0.0001);
        change=loglik-old_loglik;
        old_loglik=loglik;
        iter=iter+1;
    end % update
%     fprintf(' %d\n',iter);

    %construct polygonal line
%     fprintf('constructing curve\n');
    [cost,edgesTmp]=construct_hp(lines(:,:,1:k),lambda); % construct
    edgesTmp=optim_hp(edgesTmp,cost);            % 2-opt optimize
    edges = edgesTmp(1:(end-1), 1:(end-1));
%     edges(end,:)=[];edges(:,end)=[];
%     hold off; 
%     if size(X,2)==2
%         plot(X(:,1),X(:,2),data_opt);
%     elseif size(X,2)==3
%         plot3(X(:,1),X(:,2),X(:,3),data_opt);
%     end
%     hold on;
%     plot_pad(edges,lines(:,:,1:k));
%     fprintf('mapping points to arc-length\n');
    vertices=reshape(lines(:,:,1:k),size(X,2),2*k);
    [y,sqd]=map_to_arcl(edges,vertices,X);
    of(k, :)= [mean(sqd) log(max(y(:,1))) mean(sqd)+2*alpha*log(max(y(:,1)))];
    if (of(k,3)==min(of(1:k,3)) )
        y_best=y;
    end
end %insertion loop

[i,j]=min(of(:,3));
% fprintf('Suggested number of lines: %d\n',j);
% toc

% compute and plot RBF approximation of F: latent -> R^d , for best model

T2=sortrows([y_best(:,1) X],1);temp=[]; % order data according to latent var.
for i=2:size(X,1)                       % determine nice bandwith 's' (global)
    temp=[temp;T2(i,1)-T2(i-1,1)];
end
s=3*mean(temp);

extra = .1;      % extrapolation factor
gran = 1000;     % granularity of plot; how many positions to plot
tl = T2(end,1);  % total length of polygonal line

np=[-extra*tl:tl/gran:(1+extra)*tl];

% D contains at row i distance from data point i to evaluations points in lat.
D=repmat(np,size(X,1),1)-repmat(T2(:,1),1,size(np,2));

K=exp(-D.^2/(2*s^2)); % K is as D but contains weights
K=K./repmat(sum(K),size(X,1),1); % normalize weights
S=K'*T2(:,2:end);
% figure(5);
% if size(X,2)==3
%     plot3(S(:,1),S(:,2),S(:,3),'r');hold on;plot3(X(:,1),X(:,2),X(:,3),'.');hold off;
% else
%     plot(S(:,1),S(:,2),'r');hold on;plot(X(:,1),X(:,2),'.');hold off;
% end