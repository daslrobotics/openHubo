
A=ls;
F=textscan(A,'%s','CollectOutput',1);
files=F{1};

K1=regexpi(files,'servodata.*\.txt');

C=hsv(length(cell2mat(K1)));

clf
hold on
j=0;
for k=1:length(K1)
    if K1{k}
        j=j+1;
        data=importServoData(files{k});
        jointData{1,j}=[1:length(data.data(1,:))]';
        jointData{2,j}=data.data(1,:)';
        gains{1,j}=data.textdata{1};
        
    end
end

plot(jointData{:},jointData{1,j},data.data(2,:)')
legend(gains{:})