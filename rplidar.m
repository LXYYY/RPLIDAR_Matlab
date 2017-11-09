clear all
clc
loadlibrary('libRPLIDAR.dll','librplidar.h')
calllib('libRPLIDAR','lidarInit','//./com3',115200)

calllib('libRPLIDAR','lidarStart')
% b=zeros(1,720*5,'uint8');
%     a=libpointer('uint8Ptr',b);
% setdatatype(a,'uint8Ptr',1,64)

% for i=1:720*5
%     a(i)=libpointer('uint8Ptr');
% end
nodesZ=zeros(1,720*5);
nodesPtr=libpointer('uint8Ptr',nodesZ);

while(1)
    countPtr=libpointer('uint64Ptr',720);
    a=calllib('libRPLIDAR','lidarGetScanData',nodesPtr,countPtr,2000);
    nodesU8=get(nodesPtr,'value');
    count=get(countPtr,'value');
    for i=1:count
        sync_quality=nodesU8((i-1)*5+1);
        angle_q6_checkbit=uint16(uint16(nodesU8((i-1)*5+2))*(2^8)+uint16(nodesU8((i-1)*5+3)));
        distance_q2=uint16(uint16(nodesU8((i-1)*5+4))*(2^8)+uint16(nodesU8((i-1)*5+5)));
        nodes(i).angle_q6_checkbit=angle_q6_checkbit;
        nodes(i).syncbit=sync_quality&1;
        nodes(i).theta=(double(angle_q6_checkbit)/2)/64;
        nodes(i).distance=double(distance_q2)/4;
        nodes(i).Q=double(sync_quality)/4;
    end
end

calllib('libRPLIDAR','lidarStop')
unloadlibrary('libRPLIDAR')