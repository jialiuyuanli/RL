clear all
close all
clc

%%Initialization
L=3;%the number of leader UAVs
M=9;%the number of member UAVs
Mmax=5;%the maximal number of the member UAVs in a swarm
K=500;%the number of ground users
C=10;%type of contents
S=6;%capacity of leader UAVs
Sm=6;%capacity of member UAVs
D=1*10^7;%task size
T=200;%iteration
HL=150;% the altitude of leader UAVs
HM=120;% the altitude of member UAVs
Bw=2*10^6;%band width
BwNo=7.943282347242812*10^(-15);%interference noise
%Dis=zeros(T,M,K);%distence between GU k and UAV m at time slot t
E=zeros(T,L);%remaining energy of leader UAVs at the end of time slot t
b=zeros(T,L);%leader UAV l returns to charging station at time slot t or not
%TH=zeros();%hovering time of UAV
TS=25;%time of each slot
W=2500;%the width of target area
q=50;%side length of small grids
lq=5*q;%side length of big grids
PtGU=0.2;%transmit power of GU
PtM=2;%transmit power of member UAV
V=20;%speed of UAV
Pfv=0.5*0.6*1.225*0.05*0.503*V^3+0.012/8*1.225*0.05*0.503*300^3*0.4^3*(1+3*V^2/120^2)+1.1*20^1.5/sqrt(2*1.225*0.503)*(sqrt(1+V^4/4*4.03^4)-V^2/2*4.03^2)^0.5;
Pfo=0.012/8*1.225*0.05*0.503*300^3*0.4^3+1.1*20^1.5/sqrt(2*1.225*0.503);
EproM=Pfv*q/V+Pfo*TS;
EproM5=Pfo*TS;
EproL=Pfv*lq/V+Pfo*TS;
EproL5=Pfo*TS;
TaverageGU=zeros(T,L,4);
TaverageM=zeros(T,L,4);
Etotal=1000000;%Total energy capacity of leader UAV l(J)
Er=10000;
%c=ones(T,K,C);%GU k requests content s at time slot t or not
Tc=10;%Time cost of recharging(slot)
A=1;
B=1;
Delta1=1;
Delta2=1;
acL=zeros(T,L);%joint cruise action of leader UAV at time slot t
aeL=zeros(T,L);%joint energy action of leader UAV at time slot t
acM=zeros(T,M);%joint cruise action of member UAV at time slot t
asM=zeros(T,M);%joint swarm action of member UAV at time slot t
arM=zeros(T,M);%joint relay action of member UAV at time slot t
%sc=zeros(T,M);%environment state at time slot t for cruise learner
%se=zeros(T,M);%environment state at time slot t for energy learner
%s=zeros(T,M);%environment state for cruise learner and energy learner
%a=zeros(T,M);%joint cruise and energy action
%Ne=ones(T,M,M);%number of occurrences of state-action pair (se,ae) at time slot t
%Nc=ones(T,M,M);%number of occurrences of state-action pair (se,ae) at time slot t
%AA=factorial(C)/(factorial(C-S)*factorial(S));
rcL=zeros(T,L,4);%reward of cruise learner
reL=zeros(T,L,2);%reward of energy learner
rcM=zeros(T,M,4);%reward of cruise learner
rsM=zeros(T,M,L);%reward of swarm learner
rrM=zeros(T,M,C);%reward of relay learner
%energycon=zeros(T,M,5,16);
%ra=zeros(T,M,AA);
QcL=zeros(T,L,4);%Q value of cruise learner
QeL=zeros(T,L,2);%Q value of energy learner
QcM=zeros(T,M,4);%Q value of cruise learner
QsM=zeros(T,M,L);%Q value of swarm learner
QrM=zeros(T,M,C);%Q value of relay learner
%Qa=zeros(T,M,AA);%Q value of application learner
alphaE=zeros(T,M,M);%learning rate
betaE=zeros(T,M,M);%learning rate
alphaC=zeros(T,M,M);%learning rate
betaC=zeros(T,M,M);%learning rate
z=zeros(T,M,K,4);%UAV and GU are located in the same grid at time slot t or not
tt=zeros(1,L);
tb=ones(1,L);
%o=randi([0,1],T,K);
o=ones(T,K);
v=zeros(T,C,K);
wL=zeros(C,L);
wM=zeros(C,M);
EtranMrc=zeros(T,M,4);
EtranMrs=zeros(T,M,L);
EtranMrr=zeros(T,M,C);
EcompLrc=zeros(T,L,4);
EcompLrr=zeros(T,L,C);
EcompMrc=zeros(T,M,4);
EcompMrs=zeros(T,M,L);
EcompMrr=zeros(T,M,C);
delta=zeros(T,L,M);
phi=ones(T,M,C);
count=0;
sumnozero=0;
xi=10^(-18);
fren=3*10^9;
c=3*10^8;
fUAVL=1*10^6;
fUAVM=1*10^6;
f=zeros(1,T);
for l=1:L
    wL(:,l)=[1,1,1,1,1,1,1,1,1,1];
end
wM(:,1)=[1,1,1,1,1,1,0,0,0,0];
wM(:,2)=[0,1,1,1,1,1,1,0,0,0];
wM(:,3)=[0,0,1,1,1,1,1,1,0,0];
wM(:,4)=[0,0,0,1,1,1,1,1,1,0];
wM(:,5)=[0,0,0,0,1,1,1,1,1,1];
wM(:,6)=[1,0,0,0,0,1,1,1,1,1];
wM(:,7)=[1,1,0,0,0,0,1,1,1,1];
wM(:,8)=[1,1,1,0,0,0,0,1,1,1];
wM(:,9)=[1,1,1,1,0,0,0,0,1,1];

delta(1,1,1)=1;
delta(1,1,2)=1;
delta(1,1,3)=1;
delta(1,2,4)=1;
delta(1,2,5)=1;
delta(1,2,6)=1;
delta(1,3,7)=1;
delta(1,3,8)=1;
delta(1,3,9)=1;

for t=1:T
    for k=1:K
        vind=randsrc(1,1,[1 2 3 4 5 6 7 8 9 10;0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]);
        %vind=randsrc(1,1,[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20;0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05]);
        v(t,vind,k)=1;
    end
end
epsilon=0.1;
gamma=0.9;
LOOP=1;
sumrc=zeros(1,T);
LM=zeros(T,M,2);%position of UAV m at time slot t
Lb=zeros(L,2);
LL=zeros(T,L,2);%position of UAV m at time slot t

% LL(1,:,:)=[712.5 712.5
%     1212.5 1212.5
%     1712.5 1712.5];
% LM(1,:,:)=[662.5 662.5
%     712.5 712.5
%     762.5 762.5
%     1162.5 1162.5
%     1212.5 1212.5
%     1262.5 1262.5
%     1662.5 1662.5
%     1712.5 1712.5
%     1762.5 1762.5];
LL(1,:,:)=[725 725
    1225 1225
    1725 1725];
LM(1,:,:)=[625 625
    725 725
    825 825
    1125 1125
    1225 1225
    1325 1325
    1675 1625
    1725 1725
    1825 1825];
% LL(1,:,:)=[725 725
%     1225 1225
%     1725 1725];
% LM(1,:,:)=[625 625
%     625 725
%     725 625
%     725 725
%     725 825
%     825 725
%     825 825
%     1125 1125
%     1825 1825];
% LL(1,:,:)=[712.5 712.5
%     1212.5 1212.5
%     1712.5 1712.5];
% LM(1,:,:)=[562.5 562.5
%     712.5 712.5
%     862.5 862.5
%     1062.5 1062.5
%     1212.5 1212.5
%     1362.5 1362.5
%     1562.5 1562.5
%     1712.5 1712.5
%     1862.5 1862.5];
U=1000*[1.4299    1.2714
    1.4431    0.3423
    1.0820    0.6944
    0.0171    1.3951
    1.3049    1.8188
    1.2793    1.7948
    0.0713    1.4818
    0.0388    0.5607
    1.7308    1.5206
    0.8293    0.1667
    0.4843    1.8354
    0.9031    0.1502
    1.6930    0.8156
    0.5208    1.2950
    1.9987    0.6254
    2.1391    2.1721
    0.2643    0.6929
    1.9808    1.5827
    1.0820    1.2310
    1.0192    1.6410
    0.4035    0.5299
    2.1024    0.1780
    2.0861    1.4719
    0.3368    0.4610
    0.5296    1.2597
    0.3738    2.3199
    1.2726    2.1198
    1.3076    2.3588
    1.3373    2.0666
    1.0447    1.1997
    1.8464    1.4000
    0.8745    0.5783
    1.8820    1.2300
    1.8922    0.5714
    2.4039    1.0721
    0.8748    1.2371
    1.5728    0.0846
    1.8599    1.9724
    1.2018    1.1657
    2.4955    1.7755
    0.6978    2.2911
    2.2029    1.9073
    1.6694    0.9734
    1.5306    2.1704
    1.2612    0.2960
    2.1961    1.8594
    1.0690    0.9772
    2.4513    0.3908
    1.2771    2.4734
    2.4329    1.8944
    1.0198    0.5068
    1.7442    1.7152
    0.7683    1.0572
    1.5837    2.2149
    0.3025    0.1371
    0.3766    0.1981
    1.2407    0.2051
    1.8209    1.6394
    1.6622    2.0172
    1.3570    1.4555
    1.2345    1.2674
    1.9633    1.2756
    0.7760    0.1061
    2.3279    2.1258
    1.3134    0.9401
    0.1064    1.0494
    2.0741    1.1948
    0.6283    1.9959
    2.0498    0.9858
    1.6766    1.9422
    1.5188    2.2381
    0.0511    1.1142
    1.0640    1.0513
    0.2072    0.0905
    0.1106    1.6839
    1.6237    2.3012
    0.1826    1.6221
    2.4980    2.1710
    0.5898    0.2450
    1.4880    0.3475
    0.7098    2.4886
    0.3959    2.3771
    2.1342    1.2887
    2.0709    0.9665
    1.8946    1.7768
    0.1077    2.2213
    1.4967    1.0995
    1.1762    0.7217
    0.7035    2.1239
    1.3263    2.2370
    0.6433    1.0970
    1.8800    0.9518
    1.4387    1.0727
    1.4063    0.1086
    1.5756    1.3451
    2.0800    1.0645
    0.5501    1.4039
    1.2599    2.1227
    0.9116    1.9399
    1.4205    2.3291
    1.0798    1.2563
    2.3601    1.7250
    0.7966    1.7638
    1.6806    0.2584
    0.0199    2.3515
    1.3842    0.7020
    2.3103    0.1467
    1.6038    0.4096
    1.5318    1.3006
    1.4055    1.3265
    0.4489    2.4840
    1.6911    0.3631
    1.3227    1.8754
    0.2078    0.7300
    0.9005    1.7684
    1.9602    1.9270
    1.0960    1.5351
    1.7746    0.1277
    2.1809    1.5073
    0.2807    1.9811
    0.8287    0.3310
    0.6125    2.2956
    2.0953    1.3540
    0.6636    0.2420
    0.6939    0.9243
    0.3602    1.7479
    1.8203    0.7052
    0.7364    0.9060
    2.0526    2.2894
    0.3548    0.0075
    0.0480    1.3862
    2.2735    0.7503
    0.8043    1.5638
    0.8921    0.8153
    1.4335    2.3058
    0.6151    1.4717
    1.0625    1.1298
    0.3719    1.4644
    1.7404    0.9657
    0.2894    0.0807
    1.9649    1.4689
    1.6140    0.7173
    0.9282    1.3507
    1.5212    1.3145
    0.0573    2.3946
    1.2755    2.0330
    1.6227    0.6785
    2.4409    1.3174
    0.2095    1.1407
    1.7710    0.0158
    2.1800    1.6011
    1.0138    0.2066
    1.4141    2.1179
    1.0135    1.5391
    1.5834    1.5112
    0.4967    2.1445
    0.5914    0.6776
    2.3525    1.7945
    1.7291    0.1000
    2.3828    2.1294
    1.7126    0.6490
    2.1164    1.8278
    1.4134    0.1386
    1.5828    0.7776
    2.2352    0.2489
    2.0284    1.5659
    0.4122    0.4014
    1.3368    2.0125
    0.2859    2.4420
    1.4488    0.5383
    1.7243    1.4553
    0.0910    0.5145
    1.1090    0.2683
    1.8981    1.9658
    0.1676    1.4329
    0.3124    1.7044
    1.4997    0.6040
    1.7095    2.0941
    1.5222    1.5259
    0.4836    0.8244
    2.2718    1.2277
    0.6699    2.0696
    1.7776    1.5902
    0.0798    1.3586
    0.1508    1.8852
    2.0040    1.5320
    1.9350    2.2219
    2.1909    2.4831
    0.5398    0.8434
    1.2168    1.8263
    0.9571    2.3808
    2.0794    0.5400
    0.9541    1.3692
    1.9167    0.3609
    0.9411    1.0784
    1.2108    1.6002
    1.4157    0.4704
    1.7160    1.8433
    0.4114    1.7795
    2.2586    1.0511
    1.7478    0.6289
    2.4004    1.1176
    1.9383    1.8747
    1.5992    0.0859
    1.0922    0.6716
    1.5701    2.0588
    2.3892    1.0135
    1.6945    1.6083
    0.1150    0.2514
    2.2244    1.2854
    1.0832    1.3592
    0.2175    2.1422
    2.1394    0.8107
    1.5142    0.4006
    0.2237    0.1021
    1.3604    1.6939
    0.5807    1.6030
    0.9338    0.2947
    1.9243    1.9925
    1.3912    2.3877
    1.9320    1.9111
    2.2249    0.5103
    0.9565    1.0050
    1.0471    0.1618
    0.1516    1.6803
    2.3333    1.6557
    0.6830    2.4024
    1.0714    1.9632
    2.3139    1.8912
    0.6597    0.0076
    0.5568    0.1876
    2.3540    0.6573
    0.5352    2.4360
    1.6611    1.3463
    0.0042    0.7100
    0.1915    0.2379
    2.2185    0.7292
    2.1580    2.4187
    2.0103    0.7993
    0.5036    2.2793
    2.2271    1.0963
    1.9461    2.1533
    2.0616    2.0723
    0.8192    1.8580
    0.4149    1.4394
    0.6522    0.2725
    0.5610    1.6518
    2.0991    1.7145
    1.2531    1.4710
    0.8761    0.4073
    2.1113    0.6705
    1.0527    1.3083
    1.1509    1.8014
    1.0996    1.0497
    0.6347    2.4585
    2.4362    1.2281
    0.7786    0.2523
    1.5390    1.3261
    0.7597    2.1646
    1.9201    1.1606
    1.2308    0.5601
    1.1650    0.5335
    2.0467    2.2807
    0.2684    1.5216
    1.9023    0.4337
    2.0477    1.9442
    1.7559    1.0188
    1.7233    2.0287
    1.1736    2.4077
    2.1059    1.2705
    0.0050    0.5678
    2.0203    1.9606
    1.6768    0.7177
    1.9478    2.1157
    0.0202    2.2074
    0.9298    0.6701
    2.0675    0.7432
    1.9528    0.9691
    0.8061    2.2147
    1.0405    2.4618
    0.3608    2.0979
    0.9268    1.3075
    0.1887    1.2824
    2.0227    0.0170
    1.9814    0.3966
    1.8403    2.3676
    0.5185    1.6669
    2.1255    2.2950
    1.9281    2.2237
    0.6492    1.0698
    1.8244    2.0775
    0.3266    1.8100
    1.8516    0.1327
    0.0217    0.4979
    2.2059    2.4610
    2.4444    1.9688
    2.2350    2.3992
    1.4537    0.1119
    1.7390    1.4838
    0.5999    1.5904
    2.2378    0.6236
    1.4275    0.6964
    1.1951    0.3123
    1.4915    1.3998
    2.0083    1.7032
    0.8331    0.1818
    1.9176    2.3824
    0.9922    0.7807
    0.1505    1.4976
    1.6333    0.3254
    0.2101    0.4839
    2.1165    1.8245
    1.0571    0.8034
    0.7229    1.3273
    1.2296    1.2571
    2.2870    0.3697
    1.1406    1.9444
    0.5221    2.3928
    0.8563    0.0392
    0.2190    1.5478
    0.2656    0.7990
    2.3839    1.6947
    2.3092    0.7600
    2.0679    1.7016
    0.8299    2.4613
    0.2413    2.2717
    1.0902    0.8132
    1.9233    0.0895
    1.8263    0.8866
    0.8368    0.1318
    1.7530    2.2359
    1.4596    0.2694
    1.7420    0.3804
    0.6374    1.6967
    0.2956    0.3084
    2.3515    1.6334
    0.3924    2.2594
    0.9414    0.0761
    2.2623    1.1087
    0.2786    0.7981
    2.3564    2.0271
    2.0689    1.4279
    0.9684    1.9209
    0.2458    0.1994
    0.0178    1.7427
    1.5726    1.4606
    2.0036    1.6847
    0.6995    0.6940
    2.3640    1.9420
    1.7890    2.2138
    1.6028    0.8307
    2.1740    1.0049
    2.1590    0.0809
    0.7732    0.4533
    2.0872    1.3670
    0.8259    2.1811
    1.5521    2.4379
    0.8928    2.0635
    0.3946    1.6724
    2.3222    1.7711
    2.3892    2.4893
    0.2397    0.4342
    1.2847    1.3425
    1.0360    1.5061
    2.0445    1.0772
    1.6446    0.6483
    1.2059    1.4182
    2.1303    0.9052
    1.4605    1.0436
    1.1972    0.9724
    0.3595    1.5224
    1.7328    0.6281
    0.1656    1.8948
    0.9953    1.8032
    2.4492    1.3085
    0.3502    0.5341
    0.4346    1.7349
    1.1127    1.9172
    1.9264    0.7954
    0.9854    1.4607
    0.6032    0.8908
    0.5480    0.0128
    0.9515    1.8789
    1.9075    2.1404
    0.7381    1.9477
    1.5704    1.4310
    2.2835    0.5863
    0.8725    2.4034
    0.4545    1.8361
    0.8238    1.4098
    0.6447    2.3833
    1.2159    1.9598
    1.2889    1.2653
    0.3651    0.3289
    1.3037    0.7432
    1.4784    2.4111
    2.3493    2.2336
    0.2589    1.7351
    1.0236    2.3575
    0.6364    2.0655
    2.4032    2.1245
    1.1082    0.6093
    2.0099    2.4026
    1.4443    0.2385
    1.4424    0.7381
    1.9460    0.8627
    0.1909    0.4173
    2.3870    0.9534
    0.1439    0.8794
    2.0647    2.3833
    0.0351    1.0774
    0.4391    0.1524
    0.4545    0.7598
    0.3762    1.4195
    1.4362    2.2765
    2.3568    1.4698
    0.8451    1.2592
    0.1422    1.9925
    1.0717    1.1140
    1.8049    1.4842
    0.5860    2.0727
    0.3931    1.5762
    2.0334    1.3464
    1.9235    2.3848
    0.8701    2.0542
    0.7690    0.6275
    0.3733    2.4248
    1.6474    2.2634
    1.6437    1.5301
    1.8530    2.1433
    1.8463    1.9873
    0.6875    1.0509
    1.9775    1.5725
    1.3426    0.1497
    0.8786    0.8823
    0.0787    2.4008
    1.6519    0.9582
    1.7526    0.1746
    0.0333    1.6876
    1.0667    0.5174
    0.4404    1.7530
    2.2353    0.1487
    0.4670    0.3894
    1.9211    1.3079
    1.1197    2.0985
    0.4375    1.2598
    2.0754    1.5324
    0.9645    0.7572
    0.2457    2.1358
    2.4976    1.9076
    0.0599    1.1690
    1.0090    1.4317
    0.5321    0.5185
    1.8487    1.3687
    2.0600    1.2534
    1.7890    1.7780
    2.4190    0.2488
    0.5723    1.3055
    2.0971    0.4869
    0.2946    2.2519
    2.3138    0.0400
    1.1301    1.6898
    0.1927    0.5968
    1.2292    0.2489
    1.3841    1.1064
    2.3811    1.4760
    2.3472    0.8272
    0.8178    2.4911
    0.8022    0.3757
    1.0790    2.1015
    1.6073    2.4059
    1.1260    0.1925
    0.3093    2.3129
    1.5204    1.2386
    1.4727    0.4488
    2.2811    2.4718
    1.7391    1.4625
    1.1007    1.3247
    1.3215    1.8945
    0.1288    0.5193
    1.5597    0.8523
    2.4933    1.4358
    1.2213    2.4680
    1.2008    1.8471
    0.9674    1.1355
    2.4765    0.4078
    2.4403    1.6499
    0.5410    2.0213
    0.4897    1.3518
    0.2202    2.0806
    0.2928    2.3062
    0.6468    0.1691
    1.9550    1.8235
    0.3992    2.0181
    0.0332    0.4582
    1.7192    0.0355
    0.3000    0.0200
    0.6954    2.3523
    1.0637    1.1478
    0.2455    2.1920];
%%Iteration
for loop=1:LOOP
    t=1;
    E(1,1)=1000000;
    E(1,2)=1000000;
    E(1,3)=1000000;
while(t<T)
        %alphaE(t,m)=1/(Ne(t,m)+1);
        %betaE(t,m)=1/(Ne(t,m)+1);
        %alphaC(t,m)=1/(Nc(t,m)+1);
        %betaC(t,m)=1/(Nc(t,m)+1);
        %sumQenm=0;
        %sumQcnm=0;
    %cruise rewards of member UAVs
        for m=1:M
            for i=1:4
 %          for ni=1:16
                dM=zeros(M,K);
                dL=zeros(M,L);
                plos=zeros(M,K);
                losM=zeros(M,K);
                losL=zeros(1,M);
                alpha=9.6117;
                beta=0.1581;
                LOS=1;
                NLOS=20;
                CRatioM=zeros(M,K);
                CRatioL=zeros(1,M);
                SINRM=zeros(M,K);%ratio of signal and interference at time slot t
                SINRL=zeros(1,M);
                I=zeros(1,M);%interference
                TtranGU=zeros(M,K);
                TtranM=zeros(1,M);
                TcompM=zeros(1,M);
                TaskM=zeros(1,M);
                sumNt=zeros(1,K);
                sumzo=zeros(1,M);
                nextM=zeros(M,2);
                if i==1
                    nextM(m,1)=LM(t,m,1);
                    nextM(m,2)=LM(t,m,2)+50;
                elseif i==2
                    nextM(m,1)=LM(t,m,1);
                    nextM(m,2)=LM(t,m,2)-50;
                elseif i==3
                    nextM(m,1)=LM(t,m,1)-50;
                    nextM(m,2)=LM(t,m,2);
                elseif i==4
                    nextM(m,1)=LM(t,m,1)+50;
                    nextM(m,2)=LM(t,m,2);
                end
                for k=1:K
                    for c=1:C
                        if o(t,k)*v(t,c,k)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
                            dM(m,k)=sqrt((U(k,1)-nextM(m,1))^2+(U(k,2)-nextM(m,2))^2+HM^2); 
                            plos(m,k)=alpha*exp(-beta*(atan(HM/dM(m,k))-alpha));
                            losM(m,k)=20*log2(dM(m,k))+plos(m,k)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
                            %z(t,m,k,i)=1;
                            for kk=1:K
                                for l=1:L
                                    if kk~=k&&delta(t,l,m)==1&&sqrt((U(kk,1)-LL(t,l,1))^2)<lq/2&&sqrt((U(kk,2)-LL(t,l,2))^2)<lq/2
                                        dM(m,kk)=sqrt((U(kk,1)-LL(t,l,1))^2+(U(kk,2)-LL(t,l,2))^2+HM^2);
                                        plos(m,kk)=alpha*exp(-beta*(atan(HM/dM(m,kk))-alpha));
                                        losM(m,kk)=20*log2(dM(m,kk))+plos(m,kk)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
                                        I(1,m)=I(1,m)+PtGU*10^(-losM(m,kk)/10);
                                    end
                                end
                            end
                            SINRM(m,k)=PtGU*10^(-losM(m,k)/10)/(I(1,m)+BwNo);
                            CRatioM(m,k)=Bw*log2(1+SINRM(m,k)); 
                            TtranGU(m,k)=D/CRatioM(m,k);
                            TcompM(1,m)=TcompM(1,m)+wM(c,m)*(1-phi(t,m,c))*D/fUAVM;
                        end
                    end 
                end
                for k=1:K
                    for c=1:C 
                        for l=1:L
                            if v(t,c,k)*phi(t,m,c)*delta(t,l,m)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
                                dL(m,l)=sqrt((LL(t,l,1)-nextM(m,1))^2+(LL(t,l,2)-nextM(m,2))^2+(HL-HM)^2);
                                losL(1,m)=32.45+20*log2(fren)+20*log2(dL(m,l));
                                SINRL(1,m)=PtM*10^(-losL(1,m)/10)/BwNo;
                                CRatioL(1,m)=Bw*log2(1+SINRL(1,m));
                                TtranM(1,m)=TtranM(1,m)+D/CRatioL(1,m);
                            end
                        end
                    end
                end
                %sumTcomp=0;
                %sumTtran=0;
                
                %fprintf('sumTcomp=%e \n',sumTcomp);
                EtranMrc(t,m,i)=PtM*TtranM(1,m);
                noze=TtranGU(m,:);
                noze(noze==0)=[];
                if isempty(noze)==0
                    minTtran=min(noze);
                else
                    minTtran=0;
                end
                
                %fprintf('minTtran=%d\n',minTtran);
                if (TS-q/V-lq/V)<=0
                    rcM(t,m,i)=0;  
                    %energycon(t,m,i,ni)=0;  
                elseif TcompM(1,m)<=(TS-q/V-lq/V)
                    %rc(t,m,i,ni)=-(minTtran+sumTcomp/2); 
                    TaskM(1,m)=TcompM(1,m)*fUAVM;
                    EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
                    %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
                    if i==5
                        rcM(t,m,i)=TaskM(1,m)/(EcompMrc(t,m,i)+EtranMrc(t,m,i)+EproM5);
                    else
                        rcM(t,m,i)=TaskM(1,m)/(EcompMrc(t,m,i)+EtranMrc(t,m,i)+EproM); 
                    end
                    %rc(t,m,i,ni)=floor(Task(1,m)/D); 
                    %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
                elseif TcompM(1,m)>(TS-q/V-lq/V)
                    %rc(t,m,i,ni)=-(minTtran+(TH-minTtran)/2); 
                    TaskM(1,m)=(TS-q/V-lq/V)*fUAVM;
                    EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
                    %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
                    if i==5
                        rcM(t,m,i)=TaskM(1,m)/(EcompMrc(t,m,i)+EtranMrc(t,m,i)+EproM5);
                    else
                        rcM(t,m,i)=TaskM(1,m)/(EcompMrc(t,m,i)+EtranMrc(t,m,i)+EproM); 
                    end
                    %rc(t,m,i,ni)=floor(Task(1,m)/D); 
                    %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
                end
                
                %fprintf('rc(t,m,i,ni)=%e \n',rc(t,m,i,ni));
            end
            %clustering of member UAV 
%             for i=1:L
%                 if t==1||(t>1&&delta(t-1,i,m)*delta(t,i,m)==1)
%                     dM=zeros(M,K);
%                     dL=zeros(M,L);
%                     plos=zeros(M,K);
%                     losM=zeros(M,K);
%                     losL=zeros(1,M);
%                     alpha=9.6117;
%                     beta=0.1581;
%                     LOS=1;
%                     NLOS=20;
%                     CRatioM=zeros(M,K);
%                     CRatioL=zeros(1,M);
%                     SINRM=zeros(M,K);%ratio of signal and interference at time slot t
%                     SINRL=zeros(1,M);
%                     I=zeros(1,M);%interference
%                     TtranGU=zeros(M,K);
%                     TtranM=zeros(1,M);
%                     TcompM=zeros(1,M);
%                     TaskM=zeros(1,M);
%                     sumNt=zeros(1,K);
%                     sumzo=zeros(1,M);
%                     nextM=zeros(M,2);
%                     nextM(m,1)=LM(t,m,1);
%                     nextM(m,2)=LM(t,m,2);
%                     for k=1:K
%                         for c=1:C
%                             if o(t,k)*v(t,c,k)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
%                                 dM(m,k)=sqrt((U(k,1)-nextM(m,1))^2+(U(k,2)-nextM(m,2))^2+HM^2); 
%                                 plos(m,k)=alpha*exp(-beta*(atan(HM/dM(m,k))-alpha));
%                                 losM(m,k)=20*log2(dM(m,k))+plos(m,k)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
%                                 %z(t,m,k,i)=1;
%                                 for kk=1:K
%                                     for l=1:L
%                                         if kk~=k&&delta(t,l,m)==1&&sqrt((U(kk,1)-LL(t,l,1))^2)<lq/2&&sqrt((U(kk,2)-LL(t,l,2))^2)<lq/2
%                                             dM(m,kk)=sqrt((U(kk,1)-LL(t,l,1))^2+(U(kk,2)-LL(t,l,2))^2+HM^2);
%                                             plos(m,kk)=alpha*exp(-beta*(atan(HM/dM(m,kk))-alpha));
%                                             losM(m,kk)=20*log2(dM(m,kk))+plos(m,kk)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
%                                             I(1,m)=I(1,m)+PtGU*10^(-losM(m,kk)/10);
%                                         end
%                                     end
%                                 end
%                                 SINRM(m,k)=PtGU*10^(-losM(m,k)/10)/(I(1,m)+BwNo);
%                                 CRatioM(m,k)=Bw*log2(1+SINRM(m,k)); 
%                                 TtranGU(m,k)=D/CRatioM(m,k);
%                                 TcompM(1,m)=TcompM(1,m)+wM(c,m)*(1-phi(t,m,c))*D/fUAVM;
%                             end
%                         end
%                     end
%                     for k=1:K
%                         for c=1:C 
%                             for l=1:L
%                                 if v(t,c,k)*phi(t,m,c)*delta(t,l,m)==1&&sqrt((LL(t,l,1)-nextM(m,1))^2)<lq/2&&sqrt((LL(t,l,2)-nextM(m,2))^2)<lq/2&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
%                                      dL(m,l)=sqrt((LL(t,l,1)-nextM(m,1))^2+(LL(t,l,2)-nextM(m,2))^2+(HL-HM)^2);
%                                      losL(1,m)=20*log2(dL(m,l))+plos(m,k)*LOS+20*log2(4*3.14*fren/c);
%                                      SINRL(1,m)=PtM*10^(-losL(1,m)/10)/BwNo;
%                                      CRatioL(1,m)=Bw*log2(1+SINRL(1,m));
%                                      TtranM(1,m)=TtranM(1,m)+D/CRatioL(1,m);
%                                 end
%                             end
%                         end
%                     end
%                     %sumTcomp=0;
%                     %sumTtran=0;
%                     EtranMrs(t,m,i)=PtM*TtranM(1,m);  
%                     noze=TtranGU(m,:);
%                     noze(noze==0)=[];
%                     if isempty(noze)==0
%                         minTtran=min(noze);
%                     else
%                         minTtran=0;
%                     end
%                     %fprintf('sumTcomp=%e \n',sumTcomp);
%                     if (TS-q/V-lq/V)<=0
%                         rsM(t,m,i)=0;  
%                         %energycon(t,m,i,ni)=0;  
%                     elseif TcompM(1,m)<=(TS-q/V-lq/V)
%                         %rc(t,m,i,ni)=-(minTtran+sumTcomp/2); 
%                         TaskM(1,m)=TcompM(1,m)*fUAVM;
%                         EcompMrs(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                         %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
%                         rsM(t,m,i)=TaskM(1,m)/(EcompMrs(t,m,i)+EtranMrs(t,m,i)+EproM); 
%                         %rc(t,m,i,ni)=floor(Task(1,m)/D); 
%                         %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
%                     elseif TcompM(1,m)>(TS-q/V-lq/V)
%                         %rc(t,m,i,ni)=-(minTtran+(TH-minTtran)/2); 
%                         TaskM(1,m)=(TS-q/V-lq/V)*fUAVM;
%                         EcompMrs(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                         %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
%                         rsM(t,m,i)=TaskM(1,m)/(EcompMrs(t,m,i)+EtranMrs(t,m,i)+EproM); 
%                        %rc(t,m,i,ni)=floor(Task(1,m)/D); 
%                         %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
%                     end
%                 else
%                     dM=zeros(M,K);
%                     dL=zeros(M,L);
%                     plos=zeros(M,K);
%                     losM=zeros(M,K);
%                     losL=zeros(1,M);
%                     alpha=9.6117;
%                     beta=0.1581;
%                     LOS=1;
%                     NLOS=20;
%                     CRatioM=zeros(M,K);
%                     CRatioL=zeros(1,M);
%                     SINRM=zeros(M,K);%ratio of signal and interference at time slot t
%                     SINRL=zeros(1,M);
%                     I=zeros(1,M);%interference
%                     TtranGU=zeros(M,K);
%                     TtranM=zeros(1,M);
%                     TcompM=zeros(1,M);
%                     TaskM=zeros(1,M);
%                     sumNt=zeros(1,K);
%                     sumzo=zeros(1,M);
%                     nextM=zeros(M,2);
%                     nextM(m,1)=LL(t,i,1);
%                     nextM(m,2)=LL(t,i,2);
%                     for k=1:K
%                         for c=1:C
%                             if o(t,k)*v(t,c,k)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
%                                 dM(m,k)=sqrt((U(k,1)-nextM(m,1))^2+(U(k,2)-nextM(m,2))^2+HM^2); 
%                                 plos(m,k)=alpha*exp(-beta*(atan(HM/dM(m,k))-alpha));
%                                 losM(m,k)=20*log2(dM(m,k))+plos(m,k)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
%                                 %z(t,m,k,i)=1;           
%                                 for kk=1:K
%                                     if kk~=k&&sqrt((U(kk,1)-LL(t,i,1))^2)<lq/2&&sqrt((U(kk,2)-LL(t,i,2))^2)<lq/2
%                                         dM(m,kk)=sqrt((U(kk,1)-LL(t,i,1))^2+(U(kk,2)-LL(t,i,2))^2+HM^2);
%                                         plos(m,kk)=alpha*exp(-beta*(atan(HM/dM(m,kk))-alpha));
%                                         losM(m,kk)=20*log2(dM(m,kk))+plos(m,kk)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
%                                         I(1,m)=I(1,m)+PtGU*10^(-losM(m,kk)/10);
%                                     end
%                                 end
%                                 SINRM(m,k)=PtGU*10^(-losM(m,k)/10)/(I(1,m)+BwNo);
%                                 CRatioM(m,k)=Bw*log2(1+SINRM(m,k)); 
%                                 TtranGU(m,k)=D/CRatioM(m,k);
%                                 TcompM(1,m)=TcompM(1,m)+wM(c,m)*(1-phi(t,m,c))*D/fUAVM;
%                             end
%                         end
%                     end
%                     for k=1:K
%                         for c=1:C 
%                             for l=1:L
%                                 if v(t,c,k)*phi(t,m,c)*delta(t,l,m)==1&&sqrt((LL(t,l,1)-nextM(m,1))^2)<lq/2&&sqrt((LL(t,l,2)-nextM(m,2))^2)<lq/2&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
%                                      dL(m,l)=sqrt((LL(t,l,1)-nextM(m,1))^2+(LL(t,l,2)-nextM(m,2))^2+(HL-HM)^2);
%                                      losL(1,m)=32.45+20*log2(fren)+20*log2(dL(m,l));
%                                      SINRL(1,m)=PtM*10^(-losL(1,m)/10)/BwNo; 
%                                      CRatioL(1,m)=Bw*log2(1+SINRL(1,m));
%                                      TtranM(1,m)=TtranM(1,m)+D/CRatioL(1,m);
%                                 end
%                             end
%                         end
%                     end 
%                     %sumTcomp=0;
%                     %sumTtran=0;
%                     EtranMrs(t,m,i)=PtM*TtranM(1,m);  
%                     noze=TtranGU(m,:);
%                     noze(noze==0)=[];
%                     if isempty(noze)==0
%                         minTtran=min(noze);
%                     else
%                         minTtran=0;
%                     end
%                     %fprintf('sumTcomp=%e \n',sumTcomp);
%                     if (TS-sqrt((LM(t,m,1)-LL(t,i,1))^2+(LM(t,m,2)-LL(t,i,2))^2)/V-minTtran-lq/V)<=0
%                         rsM(t,m,i)=0;  
%                         %energycon(t,m,i,ni)=0;  
%                     elseif TcompM(1,m)<=(TS-sqrt((LM(t,m,1)-LL(t,i,1))^2+(LM(t,m,2)-LL(t,i,2))^2)/V-minTtran-lq/V)
%                         %rc(t,m,i,ni)=-(minTtran+sumTcomp/2); 
%                         TaskM(1,m)=TcompM(1,m)*fUAVM;
%                         EcompMrs(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                         %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
%                         EclusM=Pfv*sqrt((LM(t,m,1)-LL(t,i,1))^2+(LM(t,m,2)-LL(t,i,2))^2)/V+Pfo*TS;
%                         rsM(t,m,i)=TaskM(1,m)/(EcompMrs(t,m,i)+EtranMrs(t,m,i)+EclusM); 
%                         %rc(t,m,i,ni)=floor(Task(1,m)/D); 
%                         %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
%                     elseif TcompM(1,m)>(TS-sqrt((LM(t,m,1)-LL(t,i,1))^2+(LM(t,m,2)-LL(t,i,2))^2)/V-minTtran-lq/V)
%                         %rc(t,m,i,ni)=-(minTtran+(TH-minTtran)/2); 
%                         TaskM(1,m)=(TS-sqrt((LM(t,m,1)-LL(t,i,1))^2+(LM(t,m,2)-LL(t,i,2))^2)/V-minTtran-lq/V)*fUAVM;
%                         EcompMrs(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                         %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
%                         EclusM=Pfv*sqrt((LM(t,m,1)-LL(t,i,1))^2+(LM(t,m,2)-LL(t,i,2))^2)/V+Pfo*TS;
%                         rsM(t,m,i)=TaskM(1,m)/(EcompMrs(t,m,i)+EtranMrs(t,m,i)++EclusM); 
%                         %rc(t,m,i,ni)=floor(Task(1,m)/D); 
%                         %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
%                     end 
%                 end
            end
            %relay selecting of member UAV 
%             for i=1:C
%                 dM=zeros(M,K);
%                 dL=zeros(M,L);
%                 plos=zeros(M,K);
%                 losM=zeros(M,K);
%                 losL=zeros(1,M);
%                 alpha=9.6117;
%                 beta=0.1581;
%                 LOS=1;
%                 NLOS=20;
%                 CRatioM=zeros(M,K);
%                 CRatioL=zeros(1,M);
%                 SINRM=zeros(M,K);%ratio of signal and interference at time slot t
%                 SINRL=zeros(1,M);
%                 I=zeros(1,M);%interference
%                 TtranGU=zeros(M,K);
%                 TtranM=zeros(1,M);
%                 TcompM=zeros(1,M);
%                 TcompL=zeros(1,L);
%                 TaskM=zeros(1,M);
%                 sumNt=zeros(1,K);
%                 sumzo=zeros(1,M);
%                 nextM=zeros(M,2);
%                 nextM(m,2)=LM(t,m,2);
%                 nextM(m,1)=LM(t,m,1);
%                 %phi(t,m,i)=1;
%                 for k=1:K
%                     for c=1:C
%                         if o(t,k)*v(t,c,k)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
%                             dM(m,k)=sqrt((U(k,1)-nextM(m,1))^2+(U(k,2)-nextM(m,2))^2+HM^2); 
%                             plos(m,k)=alpha*exp(-beta*(atan(HM/dM(m,k))-alpha));
%                             losM(m,k)=20*log2(dM(m,k))+plos(m,k)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
%                             %z(t,m,k,i)=1;
%                             for kk=1:K
%                                 for l=1:L
%                                     if kk~=k&&delta(t,l,m)==1&&sqrt((U(kk,1)-LL(t,l,1))^2)<lq/2&&sqrt((U(kk,2)-LL(t,l,2))^2)<lq/2
%                                         dM(m,kk)=sqrt((U(kk,1)-LL(t,l,1))^2+(U(kk,2)-LL(t,l,2))^2+HM^2);
%                                         plos(m,kk)=alpha*exp(-beta*(atan(HM/dM(m,kk))-alpha));
%                                         losM(m,kk)=20*log2(dM(m,kk))+plos(m,kk)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
%                                         I(1,m)=I(1,m)+PtGU*10^(-losM(m,kk)/10);
%                                     end
%                                 end
%                             end
%                             SINRM(m,k)=PtGU*10^(-losM(m,k)/10)/(I(1,m)+BwNo);
%                             CRatioM(m,k)=Bw*log2(1+SINRM(m,k)); 
%                             TtranGU(m,k)=D/CRatioM(m,k);
%                             TcompM(1,m)=TcompM(1,m)+wM(c,m)*(1-phi(t,m,c))*D/fUAVM;
%                         end
%                     end 
%                 end
%                 for k=1:K
%                     for c=1:C 
%                         for l=1:L
%                             if v(t,c,k)*phi(t,m,c)*delta(t,l,m)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
%                                  dL(m,l)=sqrt((LL(t,l,1)-nextM(m,1))^2+(LL(t,l,2)-nextM(m,2))^2+(HL-HM)^2);
%                                  losL(1,m)=32.45+20*log2(fren)+20*log2(dL(m,l));
%                                  SINRL(1,m)=PtM*10^(-losL(1,m)/10)/BwNo; 
%                                  CRatioL(1,m)=Bw*log2(1+SINRL(1,m));
%                                  TtranM(1,m)=TtranM(1,m)+D/CRatioL(1,m);
%                                  TcompL(1,l)=TcompL(1,l)+wL(c,l)*D/fUAVL;
%                             end
%                         end
%                     end
%                 end 
%                 %sumTcomp=0;
%                 %sumTtran=0;
%                 EtranMrr(t,m,i)=PtM*TtranM(1,m);  
%                 noze=TtranGU(m,:);
%                 noze(noze==0)=[];
%                 if isempty(noze)==0
%                     minTtran=min(noze);
%                 else
%                     minTtran=0;
%                 end
% 
% %                 sumTcompM=0;
% %                 sumTtranM=0;
% %                 for mm=1:M
% %                     for k=1:K
% %                         for c=1:C
% %                             if o(t,k)*v(t,c,k)==1&&sqrt((U(k,1)-LM(t,mm,1))^2)<q/2&&sqrt((U(k,2)-LM(t,mm,2))^2)<q/2
% %                                 dM(mm,k)=sqrt((U(k,1)-LM(t,mm,1))^2+(U(k,2)-LM(t,mm,2))^2+HM^2); 
% %                                 plos(mm,k)=alpha*exp(-beta*(atan(HM/dM(m,k))-alpha));
% %                                 losM(mm,k)=20*log2(sqrt(HM^2+dM(mm,k)^2)+plos(mm,k)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS);
% %                                 %z(t,m,k,i)=1;
% %                                 for kk=1:K
% %                                     for l=1:L
% %                                         if kk~=k&&delta(t,l,mm)==1&&sqrt((U(kk,1)-LL(t,l,1))^2)<lq/2&&sqrt((U(kk,2)-LL(t,l,2))^2)<lq/2
% %                                             dM(mm,kk)=sqrt((U(kk,1)-LL(t,l,1))^2+(U(kk,2)-LL(t,l,2))^2+HM^2);
% %                                             plos(mm,kk)=alpha*exp(-beta*(atan(HM/dM(mm,kk))-alpha));
% %                                             losM(mm,kk)=20*log2(sqrt(HM^2+dM(mm,kk)^2)+plos(mm,kk)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS);
% %                                             I(1,mm)=I(1,mm)+PtGU*10^(-losM(mm,kk)/10);
% %                                         end
% %                                     end
% %                                 end
% %                                 SINRM(mm,k)=PtGU*10^(-losM(mm,k)/10)/(I(1,mm)+BwNo);
% %                                 CRatioM(mm,k)=Bw*log2(1+SINRM(mm,k)); 
% %                                 TtranGU(mm,k)=D/CRatioM(mm,k);
% %                                 sumTcompM=sumTcompM+wM(c,mm)*D/fUAVM;
% %                             end
% %                             for l=1:L
% %                                 if v(t,c,k)*phi(t,mm,c)*delta(t,l,mm)==1&&sqrt((LL(t,l,1)-LM(t,mm,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,mm,2))^2)<lq/2&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
% %                                      dL(mm,l)=sqrt((LL(t,l,1)-LM(t,mm,1))^2+(LL(t,l,2)-LM(t,mm,2))^2+(HL-HM)^2);
% %                                      losL(1,mm)=32.45+20*log2(fren)+20*log2(dL(mm,l));
% %                                      SINRL(1,mm)=PtM*10^(-losL(1,mm)/10)/BwNo; 
% %                                      CRatioL(1,mm)=Bw*log2(1+SINRL(1,mm));
% %                                      sumTtranM=sumTtranM+D/CRatioL(1,mm);
% %                                 end
% %                             end
% %                         end                     
% %                     end
% %                     
% %                     if (TS-q/V-lq/V)<=0
% %                             TaskM(1,m)=0;
% %                         elseif TcompM(1,m)<=(TS-q/V-lq/V)
% %                             TaskM(1,m)=TcompM(1,m)*fUAVM;
% %                             EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
% %                         elseif TcompM(1,m)>(TS-q/V-lq/V)
% %                             TaskM(1,m)=(TS-q/V-lq/V)*fUAVM;
% %                             EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
% %                         end
% %                         EtranMrc(t,m,i)=PtM*TtranM(1,m);
% %                             
% %                             %sumTtranM=sumTtranM+TtranM(1,mm);
% %                         end
% %                     end
% %                     TaskL=TaskL+sumTtranM*fUAVL;
% %                 end
%                 %fprintf('sumTcomp=%e \n',sumTcomp);
%                 if (TS-q/V-lq/V)<=0
%                     TaskM(1,m)=0;  
%                     TaskL=0; 
%                     %energycon(t,m,i,ni)=0;  
%                 elseif TcompL(1,l)<=(TS-q/V-lq/V)
%                     %rc(t,m,i,ni)=-(minTtran+sumTcomp/2); 
%                     TaskM(1,m)=TaskM(1,m)*fUAVM;
%                     EcompMrr(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                     TaskL=TcompL(1,l)*fUAVL;
%                     EcompLrr(t,l,i)=xi*fUAVL^2*TaskL;
%                     %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
%                     %rrM(t,m,i)=TaskM(1,m)/(EcompMrr(t,m,i)+EproM); 
%                     %rc(t,m,i,ni)=floor(Task(1,m)/D); 
%                     %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
%                 elseif TcompL(1,l)>(TS-q/V-lq/V)
%                     %rc(t,m,i,ni)=-(minTtran+(TH-minTtran)/2); 
%                     TaskM(1,m)=(TS-q/V-lq/V)*fUAVM;
%                     EcompMrr(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                     TaskL=TcompL(1,l)*fUAVL;
%                     EcompLrr(t,l,i)=xi*fUAVL^2*TaskL;
%                     %fprintf('Ecomp=%e \n',Ecomp(t,m,i));
%                     %rrM(t,m,i)=TaskM(1,m)/(EcompMrr(t,m,i)+EproM); 
%                     %rc(t,m,i,ni)=floor(Task(1,m)/D); 
%                     %energycon(t,m,i,ni)=-(Ecomp(t,m,i)+Epro);
%                 end
%                 for l=1:L 
%                     rrM(t,m,i)=rrM(t,m,i)+delta(t,l,m)*(TaskL/(EcompLrr(t,l,i)+EproL)+TaskM(1,m)/(EcompMrr(t,m,i)+EtranMrr(t,m,i)+EproM));
%                 end
%             end
   
        
        
        for l=1:L  
            %cruise of leader UAV 
            for i=1:4
                dM=zeros(M,K);
                dL=zeros(M,L);
                plos=zeros(M,K);
                losM=zeros(M,K);
                losL=zeros(1,M);
                alpha=9.6117;
                beta=0.1581;
                LOS=1;
                NLOS=20;
                CRatioM=zeros(M,K);
                CRatioL=zeros(1,M);
                SINRM=zeros(M,K);%ratio of signal and interference at time slot t
                SINRL=zeros(1,M);
                I=zeros(1,M);%interference
                TtranGU=zeros(M,K);
                TtranM=zeros(1,M);
                TcompM=zeros(1,M);
                TcompL=zeros(1,L);
                TaskM=zeros(1,M);
                sumNt=zeros(1,K);
                sumzo=zeros(1,M);
                nextL=zeros(L,2);
                if i==1
                    nextL(l,1)=LL(t,l,1);
                    nextL(l,2)=LL(t,l,2)+250;
                    for m=1:M 
                        if delta(t,l,m)==1
                            nextM(m,1)=LM(t,m,1);
                            nextM(m,2)=LM(t,m,2)+250;
                        end
                    end
                elseif i==2
                    nextL(l,1)=LL(t,l,1);
                    nextL(l,2)=LL(t,l,2)-250;
                    for m=1:M 
                        if delta(t,l,m)==1
                            nextM(m,1)=LM(t,m,1);
                            nextM(m,2)=LM(t,m,2)-250;
                        end
                    end
                elseif i==3
                    nextL(l,1)=LL(t,l,1)-250;
                    nextL(l,2)=LL(t,l,2);
                    for m=1:M 
                        if delta(t,l,m)==1
                            nextM(m,1)=LM(t,m,1)-250;
                            nextM(m,2)=LM(t,m,2);
                        end
                    end
                elseif i==4
                    nextL(l,1)=LL(t,l,1)+250;
                    nextL(l,2)=LL(t,l,2);
                    for m=1:M 
                        if delta(t,l,m)==1
                            nextM(m,1)=LM(t,m,1)+250;
                            nextM(m,2)=LM(t,m,2);
                        end
                    end
                end
                %sumTcomp=0;
                TcountGU=0;
                sumTtranGU=0;
                for k=1:K
                    for c=1:C
                        for m=1:M
                            if o(t,k)*v(t,c,k)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
                                dM(m,k)=sqrt((U(k,1)-nextM(m,1))^2+(U(k,2)-nextM(m,2))^2+HM^2); 
                                plos(m,k)=alpha*exp(-beta*(atan(HM/dM(m,k))-alpha));
                                losM(m,k)=20*log2(dM(m,k))+plos(m,k)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
                                %z(t,m,k,i)=1;
                                for kk=1:K
                                    for ll=1:L
                                        if kk~=k&&delta(t,ll,m)==1&&sqrt((U(kk,1)-nextL(ll,1))^2)<lq/2&&sqrt((U(kk,2)-nextL(ll,2))^2)<lq/2
                                            dM(m,kk)=sqrt((U(kk,1)-LL(t,ll,1))^2+(U(kk,2)-LL(t,ll,2))^2+HM^2);
                                            plos(m,kk)=alpha*exp(-beta*(atan(HM/dM(m,kk))-alpha));
                                            losM(m,kk)=20*log2(dM(m,kk))+plos(m,kk)*(LOS-NLOS)+20*log2(4*3.14*fren/c)+NLOS;
                                            I(1,m)=I(1,m)+PtGU*10^(-losM(m,kk)/10);
                                        end
                                    end
                                end
                                SINRM(m,k)=PtGU*10^(-losM(m,k)/10)/(I(1,m)+BwNo);
                                SINRL(1,m)=PtM*10^(-losM(m,k)/10)/(I(1,m)+BwNo);
                                CRatioM(m,k)=Bw*log2(1+SINRM(m,k)); 
                                TtranGU(m,k)=D/CRatioM(m,k);
                                TcompM(1,m)=TcompM(1,m)+wM(c,m)*(1-phi(t,m,c))*D/fUAVM;
                                sumTtranGU=sumTtranGU+TtranGU(m,k); 
                                TcountGU=TcountGU+1;
                                %fprintf('%d\n',CRatioM(m,k));
                            end
                        end
                    end                    
                end
                TaverageGU(t,l,i)=sumTtranGU/(1000*TcountGU);
                sumTtranM=0;
                TcountM=0;
                for m=1:M
                    for c=1:C 
                        for k=1:K
                            if v(t,c,k)*phi(t,m,c)*delta(t,l,m)==1&&sqrt((U(k,1)-nextM(m,1))^2)<q/2&&sqrt((U(k,2)-nextM(m,2))^2)<q/2
                                 %dL(m,l)=sqrt((nextL(l,1)-nextM(m,1))^2+(nextL(l,2)-nextM(m,2))^2+(HL-HM)^2);
                                 %losL(1,m)=20*log2(dL(m,l))+plos(m,k)*LOS+20*log2(4*3.14*fren/c); 
                                 %losL(1,m)=20*log2(dL(m,l))+plos(m,k)*LOS+20*log2(4*3.14*fren/c);
                                 %SINRL(1,m)=PtM*10^(-losL(1,m)/10)/BwNo; 
                                 CRatioL(1,m)=Bw*log2(1+SINRL(1,m));
                                 TtranM(1,m)=D/CRatioL(1,m);
                                 TcompL(1,l)=TcompL(1,l)+wL(c,l)*D/fUAVL;
                                 sumTtranM=sumTtranM+TtranM(1,m); 
                                 TcountM=TcountM+1;
                                 %fprintf('%d',TcountM);
                            end
                        end
                    end
                end
                if TcountM>1
                    TaverageM(t,l,i)=sumTtranM/(TcountM*1000)+(D/fUAVL+TcompL(1,l))/2;
                elseif TcountM==1
                    TaverageM(t,l,i)=sumTtranM/100+TcompL(1,l);
                else
                    TaverageM(t,l,i)=0;
                end
                %fprintf('TcompL(1,l)=%e \n',TcompL(1,l));
                noze=TtranGU(:,:);
                noze(noze==0)=[];
                if isempty(noze)==0
                    minTtran=min(noze);
                else
                    minTtran=0;
                end
                sumTaskM=0;
                sumEnergyM=0;
                for m=1:M
                    if delta(t,l,m)==1
                        if (TS-q/V-lq/V)<=0
                            TaskM(1,m)=0;
                        elseif TcompM(1,m)<=(TS-q/V-lq/V)
                            TaskM(1,m)=TcompM(1,m)*fUAVM;
                            EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
                        elseif TcompM(1,m)>(TS-q/V-lq/V)
                            TaskM(1,m)=(TS-q/V-lq/V)*fUAVM;
                            EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
                        end
                        %fprintf('%d',TtranM(1,m));
                        EtranMrc(t,m,i)=PtM*sumTtranM/1000;
                        sumTaskM=sumTaskM+TaskM(1,m);
                        sumEnergyM=sumEnergyM+EtranMrc(t,m,i)+EproM;
                        %fprintf('sumTtranM=%e \n',sumTtranM);
                    end
                end
                if (TS-q/V-lq/V)<=0
                    rcL(t,l,i)=0; 
                elseif TcompL(1,l)<=(TS-q/V-lq/V)
                    TaskL=TcompL(1,l)*fUAVL;
                    EcompLrr(t,l,i)=xi*fUAVL^2*TaskL;
                    if i==5
                        rcL(t,l,i)=TaskL/(EcompLrr(t,l,i)+EproL5+EproM5*3);
                    else
                        rcL(t,l,i)=TaskL/(EcompLrr(t,l,i)+EproL+EproM*3);
                    end
                elseif TcompL(1,l)>(TS-q/V-lq/V)
                    TaskL=(TS-q/V-lq/V)*fUAVL;
                    EcompLrr(t,l,i)=xi*fUAVL^2*TaskL;
                    if i==5
                        rcL(t,l,i)=TaskL/(EcompLrr(t,l,i)+EproL5+EproM5*3);
                    else
                        rcL(t,l,i)=TaskL/(EcompLrr(t,l,i)+EproL+EproM*3);
                    end
                end
                
%                 sumTaskM=0;
%                 sumEnergyM=0;
%                 for m=1:M
%                     if delta(t,l,m)==1
%                         if (TS-q/V-lq/V)<=0
%                             TaskM(1,m)=0;
%                         elseif TcompM(1,m)<=(TS-q/V-lq/V)
%                             TaskM(1,m)=TcompM(1,m)*fUAVM;
%                             EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                         elseif TcompM(1,m)>(TS-q/V-lq/V)
%                             TaskM(1,m)=(TS-q/V-lq/V)*fUAVM;
%                             EcompMrc(t,m,i)=xi*fUAVM^2*TaskM(1,m);
%                         end
%                         %fprintf('%d',TtranM(1,m));
%                         EtranMrc(t,m,i)=PtM*TtranM(1,m)/100;
%                         sumTaskM=sumTaskM+TaskM(1,m);
%                         sumEnergyM=sumEnergyM+EcompMrc(t,m,i)+EproM;
%                         %fprintf('EtranMrc(t,m,i)=%e \n',EtranMrc(t,m,i));
%                     end
%                 end
%                 rcL(t,l,i)=sumTaskM/(sumEnergyM+EproL);
            end
            %charging of leader UAV.1,not back;2,back       
            for i=1:2
                nextb=zeros(1,L);
                if i==1
                    nextb(1,l)=0; 
                elseif i==2
                    nextb(1,l)=1;
                end
                for c=1:C
                    existc=0;
                    for ll=1:L
                        if l~=ll&&b(t,ll)==0
                            existc=existc+wL(c,ll);
                        elseif l==ll&&b(t,ll)==0
                            existc=existc+wL(c,ll)*(1-nextb(1,l));
                        end
                    end
                    if existc==0
                        reL(t,l,i)=-10;
                        break;
                    end
                end
                if reL(t,l,i)~=-10
                    reL(t,l,i)=1-b(t,l);
                end 
            end
        end

    
    for l=1:L
        %%select action
        %Gm=po/(0.5*l^2+H^2);
        %Ptm=(2^(S*D/(Bw*TH))-1)*BwNo/Gm;
        if b(t,l)==1
            if t-tt(1,l)>=Tc
                tb(1,l)=t+1;
                b(t+1,l)=0;
                E(t+1,l)=Etotal;
                LL(t+1,l,1)=Lb(l,1);
                LL(t+1,l,2)=Lb(l,2);
            else
                b(t+1,l)=1;
                E(t+1,l)=E(t,l);
                LL(t+1,l,1)=-200;
                LL(t+1,l,2)=1000;
            end 
        elseif E(t,l)<2*Er
            tt(1,l)=t;
            b(t+1,l)=1;
            Lb(l,1)=LL(t,l,1);
            Lb(l,2)=LL(t,l,2);
            LL(t+1,l,1)=-200;
            LL(t+1,l,2)=1000;
            E(t+1,l)=E(t,l);
%             if rand<1-epsilon
%                 if t>1
%                     wL(:,l)=[0,0,0,0,0,0,0,0,0,0];
% %                     if m==1
% %                         for c=1:C
% %                             if w(c,2)==0&&w(c,3)==0&&(sum(w(:,m))<S)
% %                                 w(c,m)=1;
% %                             end
% %                         end
% %                     elseif m==2
% %                         for c=1:C
% %                             if w(c,1)==0&&w(c,3)==0&&(sum(w(:,m))<S)
% %                                 w(c,m)=1;
% %                             end
% %                         end
% %                     elseif m==3
% %                         for c=1:C
% %                             if w(c,1)==0&&w(c,2)==0&&(sum(w(:,m))<S)
% %                                 w(c,m)=1;
% %                             end
% %                         end
% %                     end
% %                     wnumber=0;
% %                     for c=1:C
% %                         if wL(c,l)==1
% %                             wnumber=wnumber+1;
% %                         end
% %                     end
%                     cnumber=zeros(1,C);
%                     for k=1:K
%                         for m=1:M
%                             for ttt=tb(1,l):tt(1,l)
%                                 for c=1:C
%                                     if sqrt((U(k,1)-LM(ttt,m,1))^2)<q/2&&sqrt((U(k,2)-LM(ttt,m,2))^2)<q/2&&v(ttt,c,k)==1
%                                         cnumber(1,c)=cnumber(1,c)+1;
%                                     end
%                                 end
%                             end
%                         end
%                     end
%                     csort=sort(cnumber(1,:),'descend');
% 
%                     for c=1:C
%                         if (csort(1,c)==cnumber(1,1))&&(sum(wL(:,l))<S)
%                             wL(1,l)=1;
%                         elseif (csort(1,c)==cnumber(1,2))&&(sum(wL(:,l))<S)
%                             wL(2,l)=1;
%                         elseif (csort(1,c)==cnumber(1,3))&&(sum(wL(:,l))<S)
%                             wL(3,l)=1;
%                         elseif (csort(1,c)==cnumber(1,4))&&(sum(wL(:,l))<S)
%                             wL(4,l)=1;
%                         elseif (csort(1,c)==cnumber(1,5))&&(sum(wL(:,l))<S)
%                             wL(5,l)=1;
%                         elseif (csort(1,c)==cnumber(1,6))&&(sum(wL(:,l))<S)
%                             wL(6,l)=1;
%                         elseif (csort(1,c)==cnumber(1,7))&&(sum(wL(:,l))<S)
%                             wL(7,l)=1;
%                         elseif (csort(1,c)==cnumber(1,8))&&(sum(wL(:,l))<S)
%                             wL(8,l)=1;
%                         elseif (csort(1,c)==cnumber(1,9))&&(sum(wL(:,l))<S)
%                             wL(9,l)=1;
%                         elseif (csort(1,c)==cnumber(1,10))&&(sum(wL(:,l))<S)
%                             wL(10,l)=1;
%                         end
%                     end
%                     if sum(wL(:,l))<S
%                         for c=1:C
%                             wL(c,l)=1;
%                             if sum(wL(:,l))==S
%                                 break;
%                             end
%                         end
%                     end
%                 end              
%             end
        elseif QcL(t,l,1)==max(max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3)),QcL(t,l,4))
            if rand<1-epsilon
                acL(t,l)=1;%up
                if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                    if QcL(t,l,2)==max(QcL(t,l,2),QcL(t,l,3))
                        acL(t,l)=2;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    elseif QcL(t,l,3)==max(QcL(t,l,2),QcL(t,l,3))
                        acL(t,l)=3;
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                    if QcL(t,l,2)==max(QcL(t,l,2),QcL(t,l,4))
                        acL(t,l)=2;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    elseif QcL(t,l,4)==max(QcL(t,l,2),QcL(t,l,4))
                        acL(t,l)=4;
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,2)+250>2500
                    if QcL(t,l,2)==max(max(QcL(t,l,2),QcL(t,l,3)),QcL(t,l,4))
                        acL(t,l)=2;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    elseif QcL(t,l,3)==max(max(QcL(t,l,2),QcL(t,l,3)),QcL(t,l,4))
                        acL(t,l)=3;
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    elseif QcL(t,l,4)==max(max(QcL(t,l,2),QcL(t,l,3)),QcL(t,l,4))
                        acL(t,l)=4;
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                else
                    LL(t+1,l,1)=LL(t,l,1);
                    LL(t+1,l,2)=LL(t,l,2)+250;
                    for m=1:M
                        if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                            LM(t+1,m,1)=LM(t,m,1);
                            LM(t+1,m,2)=LM(t,m,2)+250;
                        end
                    end
                end
                %LL(t+1,m,1)=LL(t,l,1);
                %if LL(t,l,2)+250>2500
                %    acL(t,l)=2;
                %    LL(t+1,m,2)=LL(t,l,2)-250;
                %else
                %    LL(t+1,m,2)=LL(t,l,2)+250;
                %end
                E(t+1,l)=E(t,l)-EcompLrc(t,l,1)-EproL-3*EproM-3*EproM;
            else
                acL(t,l)=ceil(rand*4);
                if acL(t,l)==1
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)+250>2500
                        a=[2,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)+250>2500
                    %    acL(t,l)=2;
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,1)-EproL-3*EproM-3*EproM;
                elseif acL(t,l)==2
                    if LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)-250<0
                        a=[1,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)-250<0
                    %    acL(t,l)=1;
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,2)-EproL-3*EproM;
                elseif acL(t,l)==3
                    if LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0
                        a=[1,2,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)-250<0
                    %    acL(t,l)=4;
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,3)-EproL-3*EproM;
                elseif acL(t,l)==4
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500
                        a=[1,2,3];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)+250>2500
                    %    acL(t,l)=3;
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,4)-EproL-3*EproM;
                end
            end
        elseif QcL(t,l,2)==max(max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3)),QcL(t,l,4))
            if rand<1-epsilon
                acL(t,l)=2;%down
                if LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                    if QcL(t,l,1)==max(QcL(t,l,1),QcL(t,l,3))
                        acL(t,l)=1;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    elseif QcL(t,l,3)==max(QcL(t,l,1),QcL(t,l,3))
                        acL(t,l)=3;
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                    if QcL(t,l,1)==max(QcL(t,l,1),QcL(t,l,4))
                        acL(t,l)=1;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    elseif QcL(t,l,4)==max(QcL(t,l,1),QcL(t,l,4))
                        acL(t,l)=4;
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,2)-250<0
                    if QcL(t,l,1)==max(max(QcL(t,l,1),QcL(t,l,3)),QcL(t,l,4))
                        acL(t,l)=1;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    elseif QcL(t,l,3)==max(max(QcL(t,l,1),QcL(t,l,3)),QcL(t,l,4))
                        acL(t,l)=3;
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    elseif QcL(t,l,4)==max(max(QcL(t,l,1),QcL(t,l,3)),QcL(t,l,4))
                        acL(t,l)=4;
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                else
                    LL(t+1,l,1)=LL(t,l,1);
                    LL(t+1,l,2)=LL(t,l,2)-250;
                    for m=1:M
                        if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                            LM(t+1,m,1)=LM(t,m,1);
                            LM(t+1,m,2)=LM(t,m,2)-250;
                        end
                    end
                end
                %LL(t+1,l,1)=LL(t,l,1);
                %if LL(t,l,2)-250<0
                %    acL(t,l)=1;
                %    LL(t+1,l,2)=LL(t,l,2)+250;
                %else
                %    LL(t+1,l,2)=LL(t,l,2)-250;
                %end
                E(t+1,l)=E(t,l)-EcompLrc(t,l,2)-EproL-3*EproM;
            else 
                acL(t,l)=ceil(rand*4);
                if acL(t,l)==1
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)+250>2500
                        a=[2,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)+250>2500
                    %    acL(t,l)=2;
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,1)-EproL-3*EproM;
                elseif acL(t,l)==2
                    if LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)-250<0
                        a=[1,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)-250<0
                    %    acL(t,l)=1;
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,2)-EproL-3*EproM;
                elseif acL(t,l)==3
                    if LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0
                        a=[1,2,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)-250<0
                    %    acL(t,l)=4;
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,3)-EproL-3*EproM;
                elseif acL(t,l)==4
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500
                        a=[1,2,3];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)+250>2500
                    %    acL(t,l)=3;
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,4)-EproL-3*EproM;
                end
            end    
        elseif QcL(t,l,3)==max(max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3)),QcL(t,l,4))
            if rand<1-epsilon
                acL(t,l)=3;%left
                if LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                    if QcL(t,l,2)==max(QcL(t,l,2),QcL(t,l,4))
                        acL(t,l)=2;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    elseif QcL(t,l,4)==max(QcL(t,l,2),QcL(t,l,4))
                        acL(t,l)=4;
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                    if QcL(t,l,1)==max(QcL(t,l,1),QcL(t,l,4))
                        acL(t,l)=1;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    elseif QcL(t,l,4)==max(QcL(t,l,1),QcL(t,l,4))
                        acL(t,l)=4;
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,1)-250<0
                    if QcL(t,l,1)==max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,4))
                        acL(t,l)=1;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    elseif QcL(t,l,2)==max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,4))
                        acL(t,l)=2;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    elseif QcL(t,l,4)==max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,4))
                        acL(t,l)=4;
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                else
                    LL(t+1,l,1)=LL(t,l,1)-250;
                    LL(t+1,l,2)=LL(t,l,2);
                    for m=1:M
                        if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                            LM(t+1,m,1)=LM(t,m,1)-250;
                            LM(t+1,m,2)=LM(t,m,2);
                        end
                    end
                end
                %if LL(t,l,1)-250<0
                %    acL(t,l)=4;
                %    LL(t+1,l,1)=LL(t,l,1)+250;
                %else
                %    LL(t+1,l,1)=LL(t,l,1)-250;
                %end
                %LL(t+1,l,2)=LL(t,l,2);
                E(t+1,l)=E(t,l)-EcompLrc(t,l,3)-EproL-3*EproM;
            else 
                acL(t,l)=ceil(rand*4);
                if acL(t,l)==1
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)+250>2500
                        a=[2,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)+250>2500
                    %    acL(t,l)=2;
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,1)-EproL-3*EproM;
                elseif acL(t,l)==2
                    if LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)-250<0
                        a=[1,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)-250<0
                    %    acL(t,l)=1;
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,2)-EproL-3*EproM;
                elseif acL(t,l)==3
                    if LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0
                        a=[1,2,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)-250<0
                    %    acL(t,l)=4;
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,3)-EproL-3*EproM;
                elseif acL(t,l)==4
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                             for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500
                        a=[1,2,3];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)+250>2500
                    %    acL(t,l)=3;
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,4)-EproL-3*EproM;
                end
            end
        elseif QcL(t,l,4)==max(max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3)),QcL(t,l,4))
            if rand<1-epsilon
                acL(t,l)=4;%right
                if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                    if QcL(t,l,2)==max(QcL(t,l,2),QcL(t,l,3))
                        acL(t,l)=2;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    elseif QcL(t,l,3)==max(QcL(t,l,2),QcL(t,l,3))
                        acL(t,l)=3;
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                    if QcL(t,l,1)==max(QcL(t,l,1),QcL(t,l,3))
                        acL(t,l)=1;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    elseif QcL(t,l,3)==max(QcL(t,l,1),QcL(t,l,3))
                        acL(t,l)=3;
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                elseif LL(t,l,1)+250>2500
                    if QcL(t,l,1)==max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3))
                        acL(t,l)=1;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    elseif QcL(t,l,2)==max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3))
                        acL(t,l)=2;
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    elseif QcL(t,l,3)==max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3))
                        acL(t,l)=3;
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                else
                    LL(t+1,l,1)=LL(t,l,1)+250;
                    LL(t+1,l,2)=LL(t,l,2);
                    for m=1:M
                        if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                            LM(t+1,m,1)=LM(t,m,1)+250;
                            LM(t+1,m,2)=LM(t,m,2);
                        end
                    end
                end
                %if LL(t,l,1)+250>2500
                %    acL(t,l)=3;
                %    LL(t+1,l,1)=LL(t,l,1)-250;
                %else
                %    LL(t+1,l,1)=LL(t,l,1)+250;
                %end
                %LL(t+1,l,2)=LL(t,l,2);
                E(t+1,l)=E(t,l)-EcompLrc(t,l,4)-EproL-3*EproM;
            else 
                acL(t,l)=ceil(rand*4);
                if acL(t,l)==1
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)+250>2500
                        a=[2,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)+250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)+250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)+250>2500
                    %    acL(t,l)=2;
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,1)-EproL-3*EproM;
                elseif acL(t,l)==2
                    if LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,2)-250<0
                        a=[1,3,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1);
                        LL(t+1,l,2)=LL(t,l,2)-250;
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1);
                                LM(t+1,m,2)=LM(t,m,2)-250;
                            end
                        end
                    end
                    %LL(t+1,l,1)=LL(t,l,1);
                    %if LL(t,l,2)-250<0
                    %    acL(t,l)=1;
                    %    LL(t+1,l,2)=LL(t,l,2)+250;
                    %else
                    %    LL(t+1,l,2)=LL(t,l,2)-250;
                    %end
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,2)-EproL-3*EproM;
                elseif acL(t,l)==3
                    if LL(t,l,1)-250<0&&LL(t,l,2)+250>2500
                        a=[2,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0&&LL(t,l,2)-250<0
                        a=[1,4];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)-250<0
                        a=[1,2,4];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==4
                            LL(t+1,l,1)=LL(t,l,1)+250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)+250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)-250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)-250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)-250<0
                    %    acL(t,l)=4;
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,3)-EproL-3*EproM;
                elseif acL(t,l)==4
                    if LL(t,l,1)+250>2500&&LL(t,l,2)+250>2500
                        a=[2,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500&&LL(t,l,2)-250<0
                        a=[1,3];
                        ind=randperm(2);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    elseif LL(t,l,1)+250>2500
                        a=[1,2,3];
                        ind=randperm(3);
                        acL(t,l)=a(ind(1));
                        if acL(t,l)==1
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)+250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)+250;
                                end
                            end
                        elseif acL(t,l)==2
                            LL(t+1,l,1)=LL(t,l,1);
                            LL(t+1,l,2)=LL(t,l,2)-250;
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1);
                                    LM(t+1,m,2)=LM(t,m,2)-250;
                                end
                            end
                        elseif acL(t,l)==3
                            LL(t+1,l,1)=LL(t,l,1)-250;
                            LL(t+1,l,2)=LL(t,l,2);
                            for m=1:M
                                if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                    LM(t+1,m,1)=LM(t,m,1)-250;
                                    LM(t+1,m,2)=LM(t,m,2);
                                end
                            end
                        end
                    else
                        LL(t+1,l,1)=LL(t,l,1)+250;
                        LL(t+1,l,2)=LL(t,l,2);
                        for m=1:M
                            if delta(t,l,m)==1&&sqrt((LL(t,l,1)-LM(t,m,1))^2)<lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)<lq/2
                                LM(t+1,m,1)=LM(t,m,1)+250;
                                LM(t+1,m,2)=LM(t,m,2);
                            end
                        end
                    end
                    %if LL(t,l,1)+250>2500
                    %    acL(t,l)=3;
                    %    LL(t+1,l,1)=LL(t,l,1)-250;
                    %else
                    %    LL(t+1,l,1)=LL(t,l,1)+250;
                    %end
                    %LL(t+1,l,2)=LL(t,l,2);
                    E(t+1,l)=E(t,l)-EcompLrc(t,l,4)-EproL-3*EproM;
                end
            end
        elseif QeL(t,l,2)==max(QeL(t,l,1),QeL(t,l,2))
                tt(1,l)=t;
                b(t+1,l)=1;
                Lb(l,1)=LL(t,l,1);
                Lb(l,2)=LL(t,l,2);
                LL(t+1,l,1)=-200;
                LL(t+1,l,2)=1000;
                LM(t+1,m,1)=LM(t,m,1);
                LM(t+1,m,2)=LM(t,m,2);
                E(t+1,l)=E(t,l);
%             if rand<1-epsilon
%                 if t>1
%                     wL(:,l)=[0,0,0,0,0,0,0,0,0,0];
% %                     if m==1
% %                         for c=1:C
% %                             if w(c,2)==0&&w(c,3)==0&&(sum(w(:,m))<S)
% %                                 w(c,m)=1;
% %                             end
% %                         end
% %                     elseif m==2
% %                         for c=1:C
% %                             if w(c,1)==0&&w(c,3)==0&&(sum(w(:,m))<S)
% %                                 w(c,m)=1;
% %                             end
% %                         end
% %                     elseif m==3
% %                         for c=1:C
% %                             if w(c,1)==0&&w(c,2)==0&&(sum(w(:,m))<S)
% %                                 w(c,m)=1;
% %                             end
% %                         end
% %                     end
% %                     wnumber=0;
% %                     for c=1:C
% %                         if wL(c,l)==1
% %                             wnumber=wnumber+1;
% %                         end
% %                     end
%                     cnumber=zeros(1,C);
%                     for k=1:K
%                         for m=1:M
%                             for ttt=tb(1,l):tt(1,l)
%                                 for c=1:C
%                                     if sqrt((U(k,1)-LM(t,m,1))^2)<q/2&&sqrt((U(k,2)-LM(t,m,2))^2)<q/2&&v(ttt,c,k)==1
%                                         cnumber(1,c)=cnumber(1,c)+1;
%                                     end
%                                 end
%                             end
%                         end
%                     end
%                     csort=sort(cnumber(1,:),'descend');
%                  %   if S>wnumber
%                     for c=1:C
%                         if (csort(1,c)==cnumber(1,1))&&(sum(wL(:,l))<S)
%                             wL(1,l)=1;
%                         elseif (csort(1,c)==cnumber(1,2))&&(sum(wL(:,l))<S)
%                             wL(2,l)=1;
%                         elseif (csort(1,c)==cnumber(1,3))&&(sum(wL(:,l))<S)
%                             wL(3,l)=1;
%                         elseif (csort(1,c)==cnumber(1,4))&&(sum(wL(:,l))<S)
%                             wL(4,l)=1;
%                         elseif (csort(1,c)==cnumber(1,5))&&(sum(wL(:,l))<S)
%                             wL(5,l)=1;
%                         elseif (csort(1,c)==cnumber(1,6))&&(sum(wL(:,l))<S)
%                             wL(6,l)=1;
%                         elseif (csort(1,c)==cnumber(1,7))&&(sum(wL(:,l))<S)
%                             wL(7,l)=1;
%                         elseif (csort(1,c)==cnumber(1,8))&&(sum(wL(:,l))<S)
%                             wL(8,l)=1;
%                         elseif (csort(1,c)==cnumber(1,9))&&(sum(wL(:,l))<S)
%                             wL(9,l)=1;
%                         elseif (csort(1,c)==cnumber(1,10))&&(sum(wL(:,l))<S)
%                             wL(10,l)=1;
%                         end
%                     end
%                     if sum(wL(:,l))<S
%                         for c=1:C
%                             wL(c,l)=1;
%                             if sum(wL(:,l))==S
%                                 break;
%                             end
%                         end
%                     end
%                 end              
%             end

        end
        
        %fprintf('f=%e',f);
    %update Q value
        
        
        
        for i=1:4
                %for n=1:M
                %    if n~=m
                %        sumQcnm=sumQcnm+(Qc(t,m,i,ni)-Qc(t,n,i,ni));
                %    end
                %end
                %Qc(t+1,m,i)=Qc(t,m,i)+b(t,m)*(-betaC(t,m)*sumQcnm+alphaC(t,m)*(rc(t,m,i)+gamma*max(Qc(t,m,2:5))-Qc(t,m,i)));
           QcL(t+1,l,i)=QcL(t,l,i)+(1-b(t,l))*0.1*(rcL(t,l,i)+gamma*max(max(max(QcL(t,l,1),QcL(t,l,2)),QcL(t,l,3)),QcL(t,l,4))-QcL(t,l,i));
        end
        for i=1:2
            if i==1
                QeL(t+1,l,i)=QeL(t,l,i)+0.1*(reL(t,l,i)+gamma*max(QeL(t,l,1),QeL(t,l,2))-QeL(t,l,i));
            end
        end
        
    end
    %cruise selection
    for m=1:M
        for l=1:L
        if delta(t,l,m)==1&&QsM(t,m,l)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))&&QcM(t,m,1)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
        %if delta(t,l,m)==1&&QcM(t,m,1)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
            if rand<1-epsilon
                acM(t,m)=1;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
                if LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2        
                    LM(t+1,m,1)=LM(t+1,m,1);
                    LM(t+1,m,2)=LM(t+1,m,2)-50;
                else
                    LM(t+1,m,1)=LM(t+1,m,1);
                    LM(t+1,m,2)=LM(t+1,m,2)+50;
                end
                %LM(t+1,m,1)=LM(t+1,m,1);
                %if LM(t,m,2)+50>LL(t,l,2)+lq/2
                %    acM(t,m)=2;
                %    LM(t+1,m,2)=LM(t,m,2)-50;
                %else
                %    LM(t+1,m,2)=LM(t,m,2)+50;
                %end
            else
                acM(t,m)=ceil(rand*4);
                if acM(t,m)==1
                    if LM(t+1,m,1)+50>LL(t,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    end
                    %LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t,m,2)+50>LL(t,l,2)+lq/2
                    %    acM(t,m)=2;
                    %    LM(t+1,m,2)=LM(t,m,2)-50;
                    %else
                    %    LM(t+1,m,2)=LM(t,m,2)+50;
                    %end
                elseif acM(t,m)==2
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    end
                    %LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t,m,2)-50<LL(t,l,2)-lq/2
                    %    acM(t,m)=1;
                    %    LM(t+1,m,2)=LM(t,m,2)+50;
                    %else
                    %    LM(t+1,m,2)=LM(t,m,2)-50;
                    %end
                elseif acM(t,m)==3
                    if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                        a=[1,2,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if LM(t,m,1)-50<LL(t,l,1)-lq/2
                    %    acM(t,m)=4;
                    %    LM(t+1,m,1)=LM(t,m,1)+50;
                    %else
                    %    LM(t+1,m,1)=LM(t,m,1)-50;
                    %end
                    %LM(t+1,m,2)=LM(t+1,m,2);
                elseif acM(t,m)==4
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                        a=[1,2,3];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if LM(t,m,1)+50>LL(t,l,1)+lq/2
                    %    acM(t,m)=3;
                    %    LM(t+1,m,1)=LM(t,m,1)-50;
                    %else
                    %    LM(t+1,m,1)=LM(t,m,1)+50;
                    %end
                    %LM(t+1,m,2)=LM(t+1,m,2);
                end
            end
        elseif delta(t,l,m)==1&&QsM(t,m,l)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))&&QcM(t,m,2)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
        %elseif delta(t,l,m)==1&&QcM(t,m,1)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
            if rand<1-epsilon
                acM(t,m)=2;%down
                if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    if QcM(t,m,1)==max(QcM(t,m,1),QcM(t,m,3))
                        acM(t,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    elseif QcM(t,m,3)==max(QcM(t,m,1),QcM(t,m,3))
                        acM(t,m)=3;
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    if QcM(t,m,1)==max(QcM(t,m,1),QcM(t,m,4))
                        acM(t,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    elseif QcM(t,m,4)==max(QcM(t,m,1),QcM(t,m,4))
                        acM(t,m)=4;
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                elseif LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    if QcM(t,m,1)==max(max(QcM(t,m,1),QcM(t,m,3)),QcM(t,m,4))
                        acM(t,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    elseif QcM(t,m,3)==max(max(QcM(t,m,1),QcM(t,m,3)),QcM(t,m,4))
                        acM(t,m)=3;
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    elseif QcM(t,m,4)==max(max(QcM(t,m,1),QcM(t,m,3)),QcM(t,m,4))
                        acM(t,m)=4;
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                else
                    LM(t+1,m,1)=LM(t+1,m,1);
                    LM(t+1,m,2)=LM(t+1,m,2)-50;
                end
                %LM(t+1,m,1)=LM(t+1,m,1);
                %if LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                %    acM(t,m)=1;
                %    LM(t+1,m,2)=LM(t+1,m,2)+50;
                %else
                %    LM(t+1,m,2)=LM(t+1,m,2)-50;
                %end
            else 
                acM(t,m)=ceil(rand*4);
                if acM(t,m)==1
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    end
                    %LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                    %    acM(t,m)=2;
                    %    LM(t+1,m,2)=LM(t+1,m,2)-50;
                    %else
                    %    LM(t+1,m,2)=LM(t+1,m,2)+50;
                    %end
                elseif acM(t,m)==2
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    end
                    %LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    %    acM(t,m)=1;
                    %    LM(t+1,m,2)=LM(t+1,m,2)+50;
                    %else
                    %    LM(t+1,m,2)=LM(t+1,m,2)-50;
                    %end
                elseif acM(t,m)==3
                    if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)<0&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                        a=[1,2,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                    %    acM(t,m)=4;
                    %    LM(t+1,m,1)=LM(t+1,m,1)+50;
                    %else
                    %    LM(t+1,m,1)=LM(t+1,m,1)-50;
                    %end
                    %LM(t+1,m,2)=LM(t+1,m,2);
                elseif acM(t,m)==4
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                        a=[1,2,3];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                    %    acM(t,m)=3;
                    %    LM(t+1,m,1)=LM(t+1,m,1)-50;
                    %else
                    %    LM(t+1,m,1)=LM(t+1,m,1)+50;
                    %end
                    %LM(t+1,m,2)=LM(t+1,m,2);
                end
            end    
        elseif delta(t,l,m)==1&&QsM(t,m,l)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))&&QcM(t,m,3)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
        %elseif delta(t,l,m)==1&&QcM(t,m,1)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
            if rand<1-epsilon
                acM(t,m)=3;%left
                if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                    if QcM(t,m,2)==max(QcM(t,m,2),QcM(t,m,4))
                        acM(t,m)=2;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    elseif QcM(t,m,4)==max(QcM(t,m,2),QcM(t,m,4))
                        acM(t,m)=4;
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    if QcM(t,m,1)==max(QcM(t,m,1),QcM(t,m,4))
                        acM(t,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    elseif QcM(t,m,4)==max(QcM(t,m,1),QcM(t,m,4))
                        acM(t,m)=4;
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                    if QcM(t,m,1)==max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,4))
                        acM(t,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    elseif QcM(t,m,2)==max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,4))
                        acM(t,m)=2;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    elseif QcM(t,m,4)==max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,4))
                        acM(t,m)=4;
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                else
                    LM(t+1,m,1)=LM(t+1,m,1)-50;
                    LM(t+1,m,2)=LM(t+1,m,2);
                end
                %if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                %    acM(t,m)=4;
                %    LM(t+1,m,1)=LM(t+1,m,1)+50;
                %else
                %    LM(t+1,m,1)=LM(t+1,m,1)-50;
                %end
                %LM(t+1,m,2)=LM(t+1,m,2);
            else 
                acM(t,m)=ceil(rand*4);
                if acM(t,m)==1
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    end
                    %LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                    %    acM(t,m)=2;
                    %    LM(t+1,m,2)=LM(t+1,m,2)-50;
                    %else
                    %    LM(t+1,m,2)=LM(t+1,m,2)+50;
                    %end
                elseif acM(t,m)==2
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    end
                    %LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    %    acM(t,m)=1;
                    %    LM(t+1,m,2)=LM(t+1,m,2)+50;
                    %else
                    %    LM(t+1,m,2)=LM(t+1,m,2)-50;
                    %end
                elseif acM(t,m)==3
                    if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                        a=[1,2,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                    %    acM(t,m)=4;
                    %    LM(t+1,m,1)=LM(t+1,m,1)+50;
                    %else
                    %    LM(t+1,m,1)=LM(t+1,m,1)-50;
                    %end
                    %LM(t+1,m,2)=LM(t+1,m,2);
                elseif acM(t,m)==4
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                        a=[1,2,3];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                    %    acM(t,m)=3;
                    %    LM(t+1,m,1)=LM(t+1,m,1)-50;
                    %else
                    %    LM(t+1,m,1)=LM(t+1,m,1)+50;
                    %end
                    %LM(t+1,m,2)=LM(t+1,m,2);
                end
            end
        elseif delta(t,l,m)==1&&QsM(t,m,l)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))&&QcM(t,m,4)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
        %elseif delta(t,l,m)==1&&QcM(t,m,1)==max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))
            if rand<1-epsilon
                acM(t,m)=4;%right
                if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                    if QcM(t,m,2)==max(QcM(t,m,2),QcM(t,m,3))
                        acM(t,m)=2;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    elseif QcM(t,m,3)==max(QcM(t,m,2),QcM(t,m,3))
                        acM(t,m)=3;
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    if QcM(t,m,1)==max(QcM(t,m,1),QcM(t,m,3))
                        acM(t,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    elseif QcM(t,m,3)==max(QcM(t,m,1),QcM(t,m,3))
                        acM(t,m)=3;
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                    if QcM(t,m,1)==max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3))
                        acM(t,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    elseif QcM(t,m,2)==max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3))
                        acM(t,m)=2;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    elseif QcM(t,m,3)==max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3))
                        acM(t,m)=3;
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                else
                    LM(t+1,m,1)=LM(t+1,m,1)+50;
                    LM(t+1,m,2)=LM(t+1,m,2);
                end
                %if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                %    acM(t,m)=3;
                %    LM(t+1,m,1)=LM(t+1,m,1)-50;
                %else
                %    LM(t+1,m,1)=LM(t+1,m,1)+50;
                %end
                %LM(t+1,m,2)=LM(t+1,m,2);
            else 
                acM(t,m)=ceil(rand*4);
                if acM(t,m)==1
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)+50;
                    end
                    LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                    %    acM(t,m)=2;
                    %    LM(t+1,m,2)=LM(t+1,m,2)-50;
                    %else
                    %    LM(t+1,m,2)=LM(t+1,m,2)+50;
                    %end
                elseif acM(t,m)==2
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2)-50;
                    end
                    %LM(t+1,m,1)=LM(t+1,m,1);
                    %if LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                    %    acM(t,m)=1;
                    %    LM(t+1,m,2)=LM(t+1,m,2)+50;
                    %else
                    %    LM(t+1,m,2)=LM(t+1,m,2)-50;
                    %end
                elseif acM(t,m)==3
                    if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,4];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                        a=[1,2,4];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==4
                            LM(t+1,m,1)=LM(t+1,m,1)+50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)-50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if LM(t+1,m,1)-50<LL(t+1,l,1)-lq/2
                    %    acM(t,m)=4;
                    %    LM(t+1,m,1)=LM(t+1,m,1)+50;
                    %else
                    %    LM(t+1,m,1)=LM(t+1,m,1)-50;
                    %end
                    %LM(t+1,m,2)=LM(t+1,m,2);
                elseif acM(t,m)==4
                    if LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)+50>LL(t+1,l,2)+lq/2
                        a=[2,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2&&LM(t+1,m,2)-50<LL(t+1,l,2)-lq/2
                        a=[1,3];
                        ind=randperm(2);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    elseif LM(t+1,m,1)+50>LL(t+1,l,1)+lq/2
                        a=[1,2,3];
                        ind=randperm(3);
                        acM(t,m)=a(ind(1));
                        if acM(t,m)==1
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)+50;
                        elseif acM(t,m)==2
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2)-50;
                        elseif acM(t,m)==3
                            LM(t+1,m,1)=LM(t+1,m,1)-50;
                            LM(t+1,m,2)=LM(t+1,m,2);
                        end
                    else
                        LM(t+1,m,1)=LM(t+1,m,1)+50;
                        LM(t+1,m,2)=LM(t+1,m,2);
                    end
                    %if L(t,m,1)+50>1000
                    %    acM(t,m)=3;
                    %    L(t+1,m,1)=L(t,m,1)-50;
                    %else
                    %    L(t+1,m,1)=L(t,m,1)+50;
                    %end
                    %L(t+1,m,2)=L(t,m,2);
                end
            end
        end
        end
        %clusting selection
            sumdelta=zeros(1,L);
            for l=1:L
                for mm=1:M
                    sumdelta(1,l)=sumdelta(1,l)+delta(t,l,mm);
                end
            end
            if delta(t,1,m)==1&&QsM(t,m,1)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))
                if rand<1-epsilon
                    asM(t,m)=1;%up 
                    delta(t+1,1,m)=1;
                    LM(t+1,m,1)=LM(t+1,m,1);
                    LM(t+1,m,2)=LM(t+1,m,2);
                else
                    asM(t,m)=ceil(rand*L);
                    if asM(t,m)==1
                        delta(t+1,1,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2);
                    elseif asM(t,m)==2
                        if sumdelta(1,2)+1>Mmax
                            delta(t+1,1,m)=1;
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,2,m)=1;
                            LM(t+1,m,1)=LL(t+1,2,1);
                            LM(t+1,m,2)=LL(t+1,2,2);
                        end
                    elseif asM(t,m)==3
                        if sumdelta(1,3)+1>Mmax
                            delta(t+1,1,m)=1;
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,3,m)=1;
                            LM(t+1,m,1)=LL(t+1,3,1);
                            LM(t+1,m,2)=LL(t+1,3,2);
                        end
                    end
                end
            elseif delta(t,2,m)==1&&QsM(t,m,2)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))
                if rand<1-epsilon
                    asM(t,m)=2;%up 
                    delta(t+1,2,m)=1;
                    LM(t+1,m,1)=LM(t+1,m,1);
                    LM(t+1,m,2)=LM(t+1,m,2);
                else
                    asM(t,m)=ceil(rand*L);
                    if asM(t,m)==2
                        delta(t+1,2,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2);
                    elseif asM(t,m)==1
                        if sumdelta(1,1)+1>Mmax
                            delta(t+1,2,m)=1;
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,1,m)=1;
                            LM(t+1,m,1)=LL(t+1,1,1);
                            LM(t+1,m,2)=LL(t+1,1,2);
                        end
                    elseif asM(t,m)==3
                        if sumdelta(1,3)+1>Mmax
                            delta(t+1,2,m)=1;
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,3,m)=1;
                            LM(t+1,m,1)=LL(t+1,3,1);
                            LM(t+1,m,2)=LL(t+1,3,2);
                        end
                    end
                end
            elseif delta(t,3,m)==1&&QsM(t,m,3)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))
                if rand<1-epsilon
                    asM(t,m)=3;%up 
                    delta(t+1,3,m)=1;
                    LM(t+1,m,1)=LM(t+1,m,1);
                    LM(t+1,m,2)=LM(t+1,m,2);
                else
                    asM(t,m)=ceil(rand*L);
                    if asM(t,m)==3
                        delta(t+1,3,m)=1;
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2);
                    elseif asM(t,m)==1
                        if sumdelta(1,1)+1>Mmax
                            delta(t+1,3,m)=1;
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,1,m)=1;
                            LM(t+1,m,1)=LL(t+1,1,1);
                            LM(t+1,m,2)=LL(t+1,1,2);
                        end
                    elseif asM(t,m)==2
                        if sumdelta(1,2)+1>Mmax
                            delta(t+1,3,m)=1;
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,2,m)=1;
                            LM(t+1,m,1)=LL(t+1,2,1);
                            LM(t+1,m,2)=LL(t+1,2,2);
                        end
                    end
                end
            elseif QsM(t,m,1)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))
                if rand<1-epsilon
                    asM(t,m)=1;%up sqrt((LL(t+1,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t+1,l,2)-LM(t,m,2))^2)>=lq/2
                    if sumdelta(1,1)+1>Mmax
                        if delta(t,2,m)==1
                            delta(t+1,2,m)=1;
                        elseif delta(t,3,m)==1
                            delta(t+1,3,m)=1;
                        elseif delta(t,1,m)==1
                            delta(t+1,1,m)=1;
                        end
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2);
                    else
                        delta(t+1,1,m)=1;
                        LM(t+1,m,1)=LL(t+1,1,1);
                        LM(t+1,m,2)=LL(t+1,1,2);
                    end
                else
                    asM(t,m)=ceil(rand*L);
                    if asM(t,m)==1
                        if sumdelta(1,1)+1>Mmax
                            if delta(t,2,m)==1
                                delta(t+1,2,m)=1;
                            elseif delta(t,3,m)==1
                                delta(t+1,3,m)=1;
                            elseif delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,1,m)=1;
                            LM(t+1,m,1)=LL(t+1,1,1);
                            LM(t+1,m,2)=LL(t+1,1,2);
                        end
                    elseif asM(t,m)==2
                        if sumdelta(1,2)+1>Mmax
                            if delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            elseif delta(t,3,m)==1
                                delta(t+1,3,m)=1;
                            elseif delta(t,2,m)==1
                            delta(t+1,2,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,2,m)=1;
                            LM(t+1,m,1)=LL(t+1,2,1);
                            LM(t+1,m,2)=LL(t+1,2,2);
                        end
                    elseif asM(t,m)==3
                        if sumdelta(1,3)+1>Mmax
                            if delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            elseif delta(t,2,m)==1
                                delta(t+1,2,m)=1;
                            elseif delta(t,3,m)==1
                            delta(t+1,3,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,3,m)=1;
                            LM(t+1,m,1)=LL(t+1,3,1);
                            LM(t+1,m,2)=LL(t+1,3,2);
                        end
                    end
                end
            elseif QsM(t,m,2)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))
                if rand<1-epsilon
                    asM(t,m)=2;%up sqrt((LL(t+1,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t+1,l,2)-LM(t,m,2))^2)>=lq/2
                    if sumdelta(1,2)+1>Mmax
                        if delta(t,1,m)==1
                            delta(t+1,1,m)=1;
                        elseif delta(t,3,m)==1
                            delta(t+1,3,m)=1;
                        elseif delta(t,2,m)==1
                            delta(t+1,2,m)=1;
                        end
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2);
                    else
                        delta(t+1,2,m)=1;
                        LM(t+1,m,1)=LL(t+1,2,1);
                        LM(t+1,m,2)=LL(t+1,2,2);
                    end
                else
                    asM(t,m)=ceil(rand*L);
                    if asM(t,m)==1
                        if sumdelta(1,1)+1>Mmax
                            if delta(t,2,m)==1
                                delta(t+1,2,m)=1;
                            elseif delta(t,3,m)==1
                                delta(t+1,3,m)=1;
                            elseif delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,1,m)=1;
                            LM(t+1,m,1)=LL(t+1,1,1);
                            LM(t+1,m,2)=LL(t+1,1,2);
                        end
                    elseif asM(t,m)==2
                        if sumdelta(1,2)+1>Mmax
                            if delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            elseif delta(t,3,m)==1
                                delta(t+1,3,m)=1;
                            elseif delta(t,2,m)==1
                            delta(t+1,2,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,2,m)=1;
                            LM(t+1,m,1)=LL(t+1,2,1);
                            LM(t+1,m,2)=LL(t+1,2,2);
                        end
                    elseif asM(t,m)==3
                        if sumdelta(1,3)+1>Mmax
                            if delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            elseif delta(t,2,m)==1
                                delta(t+1,2,m)=1;
                            elseif delta(t,3,m)==1
                            delta(t+1,3,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,3,m)=1;
                            LM(t+1,m,1)=LL(t+1,3,1);
                            LM(t+1,m,2)=LL(t+1,3,2);
                        end
                    end
                end
            elseif QsM(t,m,3)==max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))
                if rand<1-epsilon
                    asM(t,m)=3;%up sqrt((LL(t+1,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t+1,l,2)-LM(t,m,2))^2)>=lq/2
                    if sumdelta(1,3)+1>Mmax
                        if delta(t,1,m)==1
                            delta(t+1,1,m)=1;
                        elseif delta(t,2,m)==1
                            delta(t+1,2,m)=1;
                        elseif delta(t,3,m)==1
                            delta(t+1,3,m)=1;
                        end
                        LM(t+1,m,1)=LM(t+1,m,1);
                        LM(t+1,m,2)=LM(t+1,m,2);
                    else
                        delta(t+1,3,m)=1;
                        LM(t+1,m,1)=LL(t+1,3,1);
                        LM(t+1,m,2)=LL(t+1,3,2);
                    end
                else
                    asM(t,m)=ceil(rand*L);
                    if asM(t,m)==1
                        if sumdelta(1,1)+1>Mmax
                            if delta(t,2,m)==1
                                delta(t+1,2,m)=1;
                            elseif delta(t,3,m)==1
                                delta(t+1,3,m)=1;
                            elseif delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,1,m)=1;
                            LM(t+1,m,1)=LL(t+1,1,1);
                            LM(t+1,m,2)=LL(t+1,1,2);
                        end
                    elseif asM(t,m)==2
                        if sumdelta(1,2)+1>Mmax
                            if delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            elseif delta(t,3,m)==1
                                delta(t+1,3,m)=1;
                            elseif delta(t,2,m)==1
                                delta(t+1,2,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,2,m)=1;
                            LM(t+1,m,1)=LL(t+1,2,1);
                            LM(t+1,m,2)=LL(t+1,2,2);
                        end
                    elseif asM(t,m)==3
                        if sumdelta(1,3)+1>Mmax
                            if delta(t,1,m)==1
                                delta(t+1,1,m)=1;
                            elseif delta(t,2,m)==1
                                delta(t+1,2,m)=1;
                            elseif delta(t,3,m)==1
                                delta(t+1,3,m)=1;
                            end
                            LM(t+1,m,1)=LM(t+1,m,1);
                            LM(t+1,m,2)=LM(t+1,m,2);
                        else
                            delta(t+1,3,m)=1;
                            LM(t+1,m,1)=LL(t+1,3,1);
                            LM(t+1,m,2)=LL(t+1,3,2);
                        end
                    end
                end
            end
        
        %relay selection
%         if QrM(t,m,1)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=1;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,1)=1;
%         elseif QrM(t,m,2)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=2;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,2)=1;
%         elseif QrM(t,m,3)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=3;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,3)=1;
%         elseif QrM(t,m,4)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=4;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,4)=1;
%         elseif QrM(t,m,5)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=5;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,5)=1;
%         elseif QrM(t,m,6)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=6;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,6)=1;
%         elseif QrM(t,m,7)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=7;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,7)=1;
%         elseif QrM(t,m,8)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=8;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,8)=1;
%         elseif QrM(t,m,9)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=9;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,9)=1;
%         elseif QrM(t,m,10)==max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))
%             arM(t,m)=10;%up sqrt((LL(t,l,1)-LM(t,m,1))^2)>=lq/2&&sqrt((LL(t,l,2)-LM(t,m,2))^2)>=lq/2
%             phi(t+1,m,10)=1;  
%         end
        %update Q value
        for i=1:4
            for l=1:L
                if delta(t+1,l,m)*delta(t,l,m)==1
                    QcM(t+1,m,i)=QcM(t,m,i)+0.1*(rcM(t,m,i)+gamma*max(max(max(QcM(t,m,1),QcM(t,m,2)),QcM(t,m,3)),QcM(t,m,4))-QcM(t,m,i));
                else
                    QcM(t+1,m,i)=QcM(t,m,i);
                end
            end
        end
%         for i=1:L
%             QsM(t+1,m,i)=QsM(t,m,i)+0.1*(rsM(t,m,i)+gamma*max(max(QsM(t,m,1),QsM(t,m,2)),QsM(t,m,3))-QsM(t,m,i));
%         end
%         for i=1:C
%             QrM(t+1,m,i)=QrM(t,m,i)+0.1*(rrM(t,m,i)+gamma*max(max(max(max(max(max(max(max(max(QrM(t,m,1),QrM(t,m,2)),QrM(t,m,3)),QrM(t,m,4)),QrM(t,m,5)),QrM(t,m,6)),QrM(t,m,7)),QrM(t,m,8)),QrM(t,m,9)),QrM(t,m,10))-QrM(t,m,i));
%         end
    end

    %print reward
%         sumN=0;
%         sumNN=0;
%         if t>1&&b(t,m)==1
%             for k=1:K
%                 for ttt=1:t-1
%                     for mm=1:M 
%                         if ac(ttt,mm)==1
%                             sumNt(1,k)=sumNt(1,k);
%                         elseif ac(ttt,mm)==2
%                             for c=1:C
%                                 sumNt(1,k)=sumNt(1,k)+z(ttt,mm,k,2)*o(ttt,k)*v(ttt,c,k)*w(c,mm);
%                             end
%                         elseif ac(ttt,mm)==3
%                             for c=1:C
%                                 sumNt(1,k)=sumNt(1,k)+z(ttt,mm,k,3)*o(ttt,k)*v(ttt,c,k)*w(c,mm);
%                             end
%                         elseif ac(ttt,mm)==4
%                             for c=1:C
%                                 sumNt(1,k)=sumNt(1,k)+z(ttt,mm,k,4)*o(ttt,k)*v(ttt,c,k)*w(c,mm);
%                             end
%                         elseif ac(ttt,mm)==5
%                             for c=1:C
%                                 sumNt(1,k)=sumNt(1,k)+z(ttt,mm,k,5)*o(ttt,k)*v(ttt,c,k)*w(c,mm);
%                             end
%                         end
%                     end
%                 end
%                 sumN=sumN+sumNt(1,k);
%                 sumNN=sumNN+sumNt(1,k)^2;
%             end
            %fprintf('sumN=%e \n',sumN);
            %f(1,t)=sumN^2/(K*sumNN);
        %elseif t==1
            %f(1,t)=1;

        %fprintf('f=%e \n',f);
    for l=1:L
        if acL(t,l)==1
            sumrc(1,t)=sumrc(1,t)+TaverageM(t,l,1)+TaverageGU(t,l,1);
        elseif acL(t,l)==2
            sumrc(1,t)=sumrc(1,t)+TaverageM(t,l,2)+TaverageGU(t,l,2);
        elseif acL(t,l)==3
            sumrc(1,t)=sumrc(1,t)+TaverageM(t,l,3)+TaverageGU(t,l,3);
        elseif acL(t,l)==4
            sumrc(1,t)=sumrc(1,t)+TaverageM(t,l,4)+TaverageGU(t,l,4);
        else
            sumrc(1,t)=0;
            break;
        end 
    end
%     for m=1:M
%         if acM(t,l)==1
%             sumrc(1,t)=sumrc(1,t)+rcM(t,m,1);
%         elseif acM(t,l)==2
%             sumrc(1,t)=sumrc(1,t)+rcM(t,m,2);
%         elseif acM(t,l)==3
%             sumrc(1,t)=sumrc(1,t)+rcM(t,m,3);
%         elseif acM(t,l)==4
%             sumrc(1,t)=sumrc(1,t)+rcM(t,m,4);
%         end 
%     end
 %   if sumrc(1,t)>0.1
        avgsumrc=sumrc(1,t)/L;
        sumnozero=sumnozero+avgsumrc;
        %averc=sumnozero/M;
        count=count+1;
%    end
    fprintf('Immediate reward of all UAVs=%e \n',avgsumrc);
    f(1,t)=avgsumrc;
    t=t+1;
end
end
plot(linspace(1,20,20),E(1:20,1));
hold on;
plot(linspace(1,20,20),E(1:20,2));
hold on;
plot(linspace(1,20,20),E(1:20,3));
%xlabel('Number of Iterations');
%ylabel('Fairness Factor');
fprintf('Average reward of all UAVs=%e \n',sumnozero/count);

% figure(1)
% plot(1200,450,'k^-');
% hold on
% for k=1:K
%     plot(U(k,1),U(k,2),'ko-');
%     hold on
% end
%  plot(LL(5,1,1),LL(5,1,2),'r-');
%  hold on
%  plot(LL(5,2,1),LL(5,2,2),'r-');
%  hold on
%  plot(LL(5,3,1),LL(5,3,2),'r-');
%  hold on
%  plot(LM(5,1,1),LM(5,1,2),'ko-');
%  hold on
%  plot(LM(5,2,1),LM(5,2,2),'ko-');
%  hold on
%  plot(LM(5,3,1),LM(5,3,2),'ko-');
%  hold on
%  plot(LM(5,4,1),LM(5,4,2),'ko-');
%  hold on
%  plot(LM(5,5,1),LM(5,5,2),'ko-');
%  hold on
%  plot(LM(5,6,1),LM(5,6,2),'ko-');
%  hold on
%  plot(LM(5,7,1),LM(5,7,2),'ko-');
%  hold on
%  plot(LM(5,8,1),LM(5,8,2),'ko-');
%  hold on
%  plot(LM(5,9,1),LM(5,9,2),'ko-');
% figure(2)
% plot(1200,450,'k^-');
% hold on
% for k=1:K
%     plot(U(k,1),U(k,2),'ko-');
%     hold on
% end
% plot(L(:,2,1),L(:,2,2),'b-');
% figure(3)
% plot(1200,450,'k^-');
% hold on
% for k=1:K
%     plot(U(k,1),U(k,2),'ko-');
%     hold on
% end
% plot(L(:,3,1),L(:,3,2),'g-');
% legend('Trajectory of UAV 1','Trajectory of UAV 2','Trajectory of UAV 3');