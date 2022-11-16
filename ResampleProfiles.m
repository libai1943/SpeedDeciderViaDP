function [ss, vv, aa, ttt] = ResampleProfiles(s, v, a, time, nfe)
ss = [];
vv = [];
aa = [];
ttt = [];

for ii = 1 : (length(s) - 1)
    ss = [ss, linspace(s(ii), s(ii+1), nfe)];
    vv = [vv, linspace(v(ii), v(ii+1), nfe)];
    aa = [aa, linspace(a(ii), a(ii+1), nfe)];
    ttt = [ttt, linspace(time(ii), time(ii+1), nfe)];
end

NFE = length(ss);
id_list = round(linspace(1, NFE, nfe));
ss = ss(id_list);
vv = vv(id_list);
aa = aa(id_list);
tt = ttt(id_list);
end