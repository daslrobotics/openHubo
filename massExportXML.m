%Make sure to manually copy the inertia data in the CSV for the right side.
% Flip signs on Ixy and Iyz
massData=importdata('massProperties.csv')

fid=fopen('tempBodies.xml','w')

for k = 1:length(massData.textdata)
    body = massData.textdata{k};
    if body(6)=='R'
        dataVector=massData.data(k,:).*[1 1 -1 1 1 1 1 -1 -1 1];
    else
        dataVector=massData.data(k,:);
    end
    J = dataVector([1 4 6;4 2 5;6 5 3]+4);
    eig(J)

    fprintf(fid,'<Body name="%s">\n',body);
    fprintf(fid,'<Mass type="custom">\n');
    fprintf(fid,'\t<Translation>%1.9f %1.9f %1.9f</Translation>\n',dataVector(2:4));
    fprintf(fid,'\t<total>%1.9f</total>\n',dataVector(1));
    fprintf(fid,'\t<inertia>%1.9f %1.9f %1.9f %1.9f %1.9f %1.9f %1.9f %1.9f %1.9f</inertia>\n',dataVector([1 4 6 4 2 5 6 5 3]+4));

    fprintf(fid,'</Mass>\n');
    fprintf(fid,'</Body>\n');
end
fclose(fid);

